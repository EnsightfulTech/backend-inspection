"""
Modbus TCP client for the gantry PLC (Huichuan / AutoShop).

Register map — V1.2 ("3D检测桁架V1.2"), confirmed against the AutoShop variable
table on 2026-08-01:

    D902  iUp_PosNum       host -> PLC  commanded stop index, 1..iCameraNum
    D903  iUp_PosNum_Last  PLC -> host  becomes == the commanded index once the
                                        absolute move completes (bAx_AbsDone);
                                        holds the PREVIOUS index until then
    D904  iProdrdy_ToUp    PLC -> host  wall / production line in position
    D905  iUp_CameraOk     host -> PLC  capture-done handshake
    D906  iCameraNum       host -> PLC  stop count; the PLC derives the stop
                                        spacing rCameraDis from it

Preconditions that CANNOT be set or checked over Modbus (HMI / physical panel):
the axis must be enabled and homed, AUTO mode selected (bMode_Auto), and a valid
travel range set (rStratPos < rEndPos, rStratPos > 0). Together with
iCameraNum > 0 those satisfy bData_Ok; without them the PLC ignores position
commands silently. If the gantry does not move, check those first.

Note the PLC only re-triggers a move when the commanded index actually CHANGES
(the ST guard is `iUp_PosNum <> iUp_PosNum_Last`), so re-commanding the stop the
gantry is already at is a no-op by design.
"""

import asyncio

from pyModbusTCP.client import ModbusClient
from loguru import logger

from config import PLC_HOST, PLC_PORT

# --- V1.2 register map (see module docstring) ---
REG_POS_CMD = 902      # iUp_PosNum
REG_POS_DONE = 903     # iUp_PosNum_Last
REG_PROD_READY = 904   # iProdrdy_ToUp
REG_CAM_DONE = 905     # iUp_CameraOk
REG_CAMERA_NUM = 906   # iCameraNum

DEFAULT_MOVE_TIMEOUT_S = 60.0
POLL_INTERVAL_S = 0.1
# Per-request socket timeout. pyModbusTCP defaults to 30 s, which means an
# unreachable PLC blocks startup for ~30 s per call -- the UI shell waits on the
# backend, so that reads as a hang. The PLC is on the local rig network, where a
# healthy request is milliseconds; failing fast and reporting it via /health is
# far more useful than waiting.
SOCKET_TIMEOUT_S = 3.0


class PLC_D901():
    """
    V1.1 command word. NOT confirmed to exist on the V1.2 PLC -- D901 does not
    appear in the V1.2 variable table. Retained only for `stop()`, which is
    currently unused by the server.
    """
    RESET = 1
    STOP = 2
    FORWARD = 8
    BACKWARD = 16


class PLC_D904():
    """墙板就位信号 (wall in position)."""
    NOT_IN_POS = 0
    IN_POS = 1


class PLC_D905():
    """拍照完成信号 (capture finished)."""
    NOT_FINISHED = 0
    FINISHED = 1


class AsyncPLCClient():
    def __init__(self, host=PLC_HOST, port=PLC_PORT, timeout=SOCKET_TIMEOUT_S):
        self.client = ModbusClient(host=host, port=port, timeout=timeout)
        self.client.debug = True
        if not self.client.open():
            logger.error(f"PLC: failed to open Modbus connection to {host}:{port}")
        else:
            logger.success(f"PLC: connected to {host}:{port}")
        # cached so move_to() can reject out-of-range indices immediately
        self._camera_num = self.read_camera_num()
        if self._camera_num is not None:
            logger.info(f"PLC: iCameraNum (D{REG_CAMERA_NUM}) = {self._camera_num} "
                        f"-- the capture loop must not command a stop index above this")

    # ------------------------------------------------------------------ #
    # low-level helpers
    # ------------------------------------------------------------------ #
    def _read_reg(self, addr):
        """Read one holding register. Returns int, or None if the read failed."""
        regs = self.client.read_holding_registers(addr, 1)
        if not regs:
            logger.warning(f"PLC: read of D{addr} failed (connection lost?)")
            return None
        return regs[0]

    def read_camera_num(self):
        """Current iCameraNum (stop count) the PLC is dividing its travel into."""
        return self._read_reg(REG_CAMERA_NUM)

    def set_camera_num(self, n: int) -> bool:
        """
        Set iCameraNum. The PLC recomputes the stop spacing rCameraDis from this,
        so it must match the number of capture positions the caller intends to
        visit, or the stops will not land where the calibration expects.
        """
        self.client.write_single_register(REG_CAMERA_NUM, n)
        rb = self.read_camera_num()
        if rb != n:
            logger.error(f"PLC: iCameraNum readback D{REG_CAMERA_NUM}={rb} != {n}")
            return False
        self._camera_num = rb
        logger.info(f"PLC: iCameraNum set to {n}")
        return True

    # ------------------------------------------------------------------ #
    # motion
    # ------------------------------------------------------------------ #
    async def reset(self):
        """
        Command the gantry back to the FIRST stop.

        Note this is not a fault reset -- on both V1.1 and V1.2 it simply writes
        index 1 to the position-command register. The V1.2 axis fault reset
        (bAx_CanRst) is driven internally by the PLC and is not host-writable.
        """
        logger.info("PLC: commanding return to stop 1")
        self.client.write_single_register(REG_POS_CMD, 1)
        await asyncio.sleep(0.3)

    async def move_to(self, pos_idx: int, timeout_s: float = DEFAULT_MOVE_TIMEOUT_S):
        """Command an absolute move to `pos_idx` and wait until it completes."""
        # The PLC only acts on `1 <= iUp_PosNum <= iCameraNum`; anything outside
        # that is ignored silently, which would otherwise burn the full timeout.
        if self._camera_num is not None and not (1 <= pos_idx <= self._camera_num):
            raise ValueError(
                f"PLC: stop index {pos_idx} is outside 1..{self._camera_num} "
                f"(iCameraNum, D{REG_CAMERA_NUM}); the PLC would ignore it. Either the "
                f"capture loop wants more stops than the PLC is configured for, or "
                f"iCameraNum needs setting (set_camera_num).")
        # D903 is not cleared between runs, so it may already equal pos_idx. That
        # makes the command a no-op (the PLC acts only when iUp_PosNum <>
        # iUp_PosNum_Last) AND makes the wait below pass instantly having moved
        # nothing -- a silent false "arrival" that would let a capture fire at the
        # wrong place. Detect and report it rather than trusting the wait.
        before = self._read_reg(REG_POS_DONE)
        if before == pos_idx:
            logger.warning(
                f"PLC: D{REG_POS_DONE} already reads {pos_idx} before commanding; the "
                f"PLC will treat this as a no-op and 'arrival' cannot be trusted. "
                f"(Stale from a previous run - an AUTO-mode edge clears it.)")

        logger.info(f"PLC: commanding move to stop {pos_idx} (D{REG_POS_CMD})")
        self.client.write_single_register(REG_POS_CMD, pos_idx)
        await self.wait_for_inpos(pos_idx, timeout_s=timeout_s, commanded_from=before)

    async def wait_for_inpos(self, pos_idx: int,
                             timeout_s: float = DEFAULT_MOVE_TIMEOUT_S,
                             commanded_from=None):
        """
        Block until the PLC reports arrival at `pos_idx`.

        D903 (iUp_PosNum_Last) holds the PREVIOUS index until the move finishes,
        so "arrived" means D903 == the index we commanded. Testing D903 != 0 is
        NOT sufficient: it is already non-zero after the first move of a run and
        would return immediately, letting a capture fire mid-travel.

        `commanded_from` is D903's value before the command was written. If it
        already equalled pos_idx the arrival is unverifiable -- see move_to().
        """
        loop = asyncio.get_event_loop()
        t0 = loop.time()
        await asyncio.sleep(POLL_INTERVAL_S)
        while True:
            in_pos = self._read_reg(REG_POS_DONE)
            if in_pos == pos_idx:
                if commanded_from == pos_idx:
                    logger.warning(
                        f"PLC: 'arrived' at stop {pos_idx} but D{REG_POS_DONE} already "
                        f"held that value - no movement was verified")
                else:
                    logger.info(f"PLC: arrived at stop {pos_idx}")
                return
            if loop.time() - t0 > timeout_s:
                raise TimeoutError(
                    f"PLC: stop {pos_idx} not reached within {timeout_s}s "
                    f"(D{REG_POS_CMD}={pos_idx} accepted, D{REG_POS_DONE}={in_pos}). "
                    f"The command register holds the right value, so the PLC is not "
                    f"acting on it. Check in AutoShop (none of this is visible over "
                    f"Modbus): AUTO mode (bMode_Auto), standstill (bAxSt_StandStill), "
                    f"no servo/axis error, and bTestAbs not stuck TRUE - it is ORed "
                    f"into the bAx_AutoStrat timer and suppresses the rising edge "
                    f"MC_MoveAbsolute needs.")
            await asyncio.sleep(POLL_INTERVAL_S)

    async def wait_for_production_line(self, timeout_s: float = None):
        """
        Block until the PLC signals a wall is in position.

        `timeout_s=None` waits indefinitely, which is the intended production
        behaviour -- the line may legitimately take a long time to deliver.
        """
        loop = asyncio.get_event_loop()
        t0 = loop.time()
        await asyncio.sleep(POLL_INTERVAL_S)
        while True:
            prodline_in_pos = self._read_reg(REG_PROD_READY)
            if prodline_in_pos == PLC_D904.IN_POS:
                logger.info("PLC: production line in position!")
                return
            if timeout_s is not None and loop.time() - t0 > timeout_s:
                raise TimeoutError(
                    f"PLC: no wall-in-position signal within {timeout_s}s "
                    f"(D{REG_PROD_READY}={prodline_in_pos}).")
            await asyncio.sleep(POLL_INTERVAL_S)

    # ------------------------------------------------------------------ #
    # handshake
    # ------------------------------------------------------------------ #
    def write_capture_finished(self):
        self.client.write_single_register(REG_CAM_DONE, PLC_D905.FINISHED)

    def stop(self):
        """
        V1.1 stop command. D901 is NOT in the V1.2 variable table, so this is
        very likely a no-op (or worse) on the current PLC -- verify before use.
        Currently not called anywhere in the server.
        """
        logger.warning("PLC: stop() uses V1.1 register D901, unverified on V1.2")
        self.client.write_single_register(901, PLC_D901.STOP)


if __name__ == "__main__":
    c = AsyncPLCClient()
    asyncio.run(c.move_to(4))
