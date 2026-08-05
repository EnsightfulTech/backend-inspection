"""
Hardware manager for the inspection project.
Connect to PLC and RVC cameras.
If RUN_SIMULATION is True, the system will not connect to PLC and RVC cameras.

Initialisation is DELIBERATELY non-fatal: the server must always start so the UI
can load, show what is broken, and offer a retry. Each subsystem (PLC, cameras)
is brought up independently and its failure recorded in `status()` rather than
raised. Use `init()` to retry after fixing something on the rig.
"""

import asyncio
import datetime
from pathlib import Path

from loguru import logger

from backend.utils import toast_info
from config import (RUN_SIMULATION, SIMULATION_DATA_DIR, PLC_WAIT_FOR_WALL,
                    NUM_CAPTURE_POSITIONS, GET_MODEL_FROM_PLC, PLC_HOST, PLC_PORT)

if not RUN_SIMULATION:
    from .plc_backend.async_plc_client import AsyncPLCClient
    from .rvc_cameras.async_rvc import AsyncRVCXCameras
    # NOTE: snap7_client is intentionally NOT imported here. That module opens its
    # S7 connection at import time, so importing it with the wall-model PLC
    # unreachable raises and would take the whole server down at startup -- even
    # when GET_MODEL_FROM_PLC is False and it is never used. It is imported lazily
    # in read_wall_index_and_model() instead.


class HardwareNotReady(RuntimeError):
    """Raised when an operation needs hardware that failed to initialise."""


class HardwareManager:
    def __init__(self):
        self.plc_client = None
        self.rvc_client = None
        self.plc_error = None
        self.camera_error = None
        self.initialized_at = None
        self.init()

    # ------------------------------------------------------------------ #
    # bring-up / retry
    # ------------------------------------------------------------------ #
    def init(self) -> dict:
        """
        (Re)initialise hardware. Never raises -- per-subsystem failures are
        recorded and surfaced through status(). Safe to call again at runtime,
        which is what the UI's "retry connection" action does.
        """
        if RUN_SIMULATION:
            logger.info("【启动】模拟模式，跳过硬件连接")
            self.plc_client = None
            self.rvc_client = None
            self.plc_error = None
            self.camera_error = None
            self.initialized_at = datetime.datetime.now()
            toast_info("硬件：模拟模式")
            return self.status()

        logger.info("Running in real hardware mode...")
        self._init_plc()
        self._init_cameras()
        self.initialized_at = datetime.datetime.now()

        if self.plc_error or self.camera_error:
            broken = ", ".join(x for x in ("PLC" if self.plc_error else None,
                                           "相机" if self.camera_error else None) if x)
            logger.error(f"【启动】硬件初始化出错：{broken}")
            toast_info(f"硬件：{broken} 初始化失败")
        else:
            logger.success("【启动】硬件初始化完成（PLC + 相机均已连接）")
            toast_info("硬件：初始化完成")
        return self.status()

    def _init_plc(self):
        self.plc_error = None
        logger.info(f"【启动】正在连接 PLC（Modbus, {PLC_HOST}:{PLC_PORT}）…")
        try:
            self.plc_client = AsyncPLCClient()
            if not self.plc_client.client.is_open:
                raise ConnectionError(
                    f"Modbus connection to {PLC_HOST}:{PLC_PORT} is not open")

            # iCameraNum (D906) is non-retentive: it resets to 0 on every PLC
            # power cycle, and while it is 0 the PLC's bData_Ok guard is false,
            # so it silently ignores ALL position commands. Push our authoritative
            # stop count so the rig is usable straight after a power cycle.
            if not self.plc_client.set_camera_num(NUM_CAPTURE_POSITIONS):
                raise RuntimeError(
                    f"could not set iCameraNum to {NUM_CAPTURE_POSITIONS}; the gantry "
                    f"will not move while it is 0 (bData_Ok stays false)")
            logger.success("【启动】PLC 已连接")
        except Exception as e:
            self.plc_error = str(e)
            self.plc_client = None
            logger.error(f"【启动】PLC 连接失败：{e}")

    def _init_cameras(self):
        self.camera_error = None
        logger.info("【启动】正在初始化 RVC 双目相机…")
        try:
            self.rvc_client = AsyncRVCXCameras()
            if self.rvc_client.system_init() == 1:
                raise RuntimeError(
                    "RVC camera init failed -- check both cameras are powered and "
                    "on the GigE link, that their serials match CAM_SN_DICT, and "
                    "that Windows Firewall is off (it blocks the GigE stream)")
            logger.success("【启动】相机已连接")
        except Exception as e:
            self.camera_error = str(e)
            self.rvc_client = None
            logger.error(f"【启动】相机初始化失败：{e}")

    # ------------------------------------------------------------------ #
    # status
    # ------------------------------------------------------------------ #
    @property
    def ready(self) -> bool:
        """True when everything needed to run a capture is available."""
        if RUN_SIMULATION:
            return True
        return self.plc_client is not None and self.rvc_client is not None

    def status(self) -> dict:
        """JSON-able snapshot for /health and the UI's connection panel."""
        plc = {
            "ok": RUN_SIMULATION or self.plc_client is not None,
            "simulated": RUN_SIMULATION,
            "host": None if RUN_SIMULATION else f"{PLC_HOST}:{PLC_PORT}",
            "error": self.plc_error,
            "camera_num": None,
        }
        if self.plc_client is not None:
            try:
                plc["camera_num"] = self.plc_client.read_camera_num()
            except Exception as e:      # never let a status read break /health
                plc["error"] = f"register read failed: {e}"
                plc["ok"] = False

        cameras = {
            "ok": RUN_SIMULATION or self.rvc_client is not None,
            "simulated": RUN_SIMULATION,
            "error": self.camera_error,
        }

        return {
            "ready": self.ready,
            "simulation": RUN_SIMULATION,
            "expected_stops": NUM_CAPTURE_POSITIONS,
            "initialized_at": (self.initialized_at.isoformat()
                               if self.initialized_at else None),
            "plc": plc,
            "cameras": cameras,
        }

    def _require_plc(self):
        if RUN_SIMULATION:
            return
        if self.plc_client is None:
            raise HardwareNotReady(f"PLC not connected: {self.plc_error}")

    def _require_cameras(self):
        if RUN_SIMULATION:
            return
        if self.rvc_client is None:
            raise HardwareNotReady(f"cameras not initialised: {self.camera_error}")

    # ------------------------------------------------------------------ #
    # operations
    # ------------------------------------------------------------------ #
    async def reset(self):
        if RUN_SIMULATION:
            return
        self._require_plc()
        await self.plc_client.reset()

    async def move_to_and_capture(self, pos_idx: int):
        if RUN_SIMULATION:
            logger.warning(f"sending experimental data, current step: {pos_idx}")

            await asyncio.sleep(2)
            # get the str(pose_idx).zfill(2) named folder's image
            folder = Path(SIMULATION_DATA_DIR) / f"{str(pos_idx).zfill(2)}"
            left_path = folder / "left"
            right_path = folder / "right"
            img_path_l = left_path / "Image.png"
            img_path_r = right_path / "Image.png"
            pcd_path_l = left_path / "PointCloud.ply"
            pcd_path_r = right_path / "PointCloud.ply"
            depth_path_l = left_path / "Depth.tif"
            depth_path_r = right_path / "Depth.tif"
            # CHECK IF THE FILE EXISTS
            if not img_path_l.exists() or not img_path_r.exists() \
                or not pcd_path_l.exists() or not pcd_path_r.exists() \
                or not depth_path_l.exists() or not depth_path_r.exists():
                logger.error(f"Image files not found in {folder}")
            return (img_path_l, pcd_path_l, depth_path_l), (img_path_r, pcd_path_r, depth_path_r)
        else:
            self._require_plc()
            self._require_cameras()
            await self.plc_client.move_to(pos_idx)
            result = await self.rvc_client.capture_dual(pos_idx)
            return result

    async def plc_move_to(self, pos_idx: int):
        if RUN_SIMULATION:
            await asyncio.sleep(1)
        else:
            self._require_plc()
            await self.plc_client.move_to(pos_idx)

    async def plc_wait_for_prod_line(self):
        self._require_plc()
        await self.plc_client.wait_for_production_line()

    def set_capture_saving_path(self, save_path):
        if RUN_SIMULATION:
            logger.info("Simulaton setting saving path...")
        else:
            self._require_cameras()
            self.rvc_client.set_save_path(save_path)

    async def write_capture_finished(self):
        if PLC_WAIT_FOR_WALL:
            self._require_plc()
            self.plc_client.write_capture_finished()
            await asyncio.sleep(1)

    async def read_wall_index_and_model(self):
        if RUN_SIMULATION:
            return 1, "test_test"
        if not GET_MODEL_FROM_PLC:
            raise HardwareNotReady(
                "GET_MODEL_FROM_PLC is False; the wall index/model PLC is not in use")
        # Imported lazily: snap7_client connects at import time, so importing it
        # eagerly would take the server down when that PLC is unreachable.
        from .plc_backend.snap7_client import read_wall_index_and_model
        return read_wall_index_and_model()
