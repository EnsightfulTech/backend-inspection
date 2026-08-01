"""
capture_calib.py — drive the gantry + dual RVC cameras to collect a MULTI-PASS
calibration dataset in the pass-major layout that `calibrate_rig.py` consumes.

    calib_root/
      pass_00/ 01/{left,right}/{Image.png,PointCloud.ply,Depth.tif}
               02/...
               ...
      pass_01/ 01/...        (operator re-poses the boards between passes)
      ...

It reuses the backend hardware clients (AsyncRVCXCameras, AsyncPLCClient) but does
its OWN move-and-wait, because the backend `AsyncPLCClient.wait_for_inpos()` is not
safe for the V1.2 PLC (see note below). Production code is left untouched.

------------------------------------------------------------------------------
PLC V1.2 interface (from the 3D检测桁架V1.2 variable list + main/axisdeal ST)
------------------------------------------------------------------------------
  D902  iUp_PosNum       (write) commanded stop index, 1..iCameraNum
  D903  iUp_PosNum_Last  (read)  becomes == iUp_PosNum when the absolute move
                                  completes (bAx_AbsDone); holds the PREVIOUS index
                                  until then -> we must wait for D903 == commanded.
  D905  iUp_Camera0k     (write) capture-done handshake (1). Optional in calibration
                                  (no production line running); written for parity.
  D906  iCameraNum       (write) stop count. PLC recomputes rCameraDis =
                                  (rEndPos-rStratPos)/iCameraNum from it. This script
                                  writes it at startup from --n-stops.

  NOT exposed over Modbus (set on the HMI, retentive): rStratPos / rEndPos (travel
  range). rAx_RealPos is not exposed either, so per-stop position is the nominal
  rStratPos + (n-1)*rCameraDis.

OPERATOR PRECONDITIONS before running (cannot be set/checked over Modbus):
  * Axis enabled and homed (bBT_Home), no axis/servo error.
  * AUTO mode selected (bBT_Mode / X2) -> bMode_Auto TRUE.
  * HMI has valid rStratPos < rEndPos (rStratPos > 0). iCameraNum is written by this
    script over Modbus (D906) from --n-stops, so together with a valid travel range
    that satisfies bData_Ok.
If the gantry does not move, check these first.
------------------------------------------------------------------------------

This script imports PyRVC (via the backend camera client) and talks to the PLC,
so it only runs on the rig PC. It is NOT run offline.
"""

import argparse
import asyncio
import json
import os
import sys
import time
from pathlib import Path

# make the repo root importable whether run as `python calib/capture_calib.py`
# or `python -m calib.capture_calib` (so `backend.*` resolves).
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Modbus holding-register addresses (same as the proven backend client; D-numbers
# confirmed against the V1.2 variable list).
REG_POS_CMD = 902     # iUp_PosNum      (write commanded index)
REG_POS_DONE = 903    # iUp_PosNum_Last (read; == commanded when move done)
REG_CAM_DONE = 905    # iUp_Camera0k    (write capture-done)
REG_CAMERA_NUM = 906  # iCameraNum      (write stop count; PLC derives rCameraDis)


def log(msg):
    print(f"[capture_calib] {msg}", flush=True)


async def move_to_stop(plc, idx, timeout_s=60.0, poll_s=0.1):
    """
    Command an absolute move to stop `idx` and wait until it truly completes.

    Unlike the backend's wait_for_inpos() (which breaks on D903 != 0 and so returns
    stale-true immediately after the first move), this waits for D903 == idx.

    D903 (iUp_PosNum_Last) is NOT cleared between runs -- only an AUTO-mode
    rising edge zeroes it. So it can already equal `idx` before we command
    anything, in which case: (a) the PLC's own guard `iUp_PosNum <>
    iUp_PosNum_Last` makes the command a no-op, and (b) a naive wait would
    report "arrived" instantly having moved nothing. Both are called out below
    rather than silently passing, because a fake arrival at stop 1 makes the
    real failure look like it happens at stop 2.
    """
    before = plc.client.read_holding_registers(REG_POS_DONE, 1)
    before = before[0] if before else None
    stale = (before == idx)
    if stale:
        log(f"  !! D903 is ALREADY {idx} before commanding. The PLC only acts when "
            f"iUp_PosNum <> iUp_PosNum_Last, so this move is a no-op -- the gantry "
            f"will NOT move, and 'arrived' below proves nothing. Left over from a "
            f"previous run; toggle the mode switch to AUTO to clear it.")

    plc.client.write_single_register(REG_POS_CMD, idx)
    log(f"  -> commanded stop {idx} (D902={idx}, D903 was {before}); "
        f"waiting for D903 == {idx} ...")
    t0 = time.time()
    while True:
        done = plc.client.read_holding_registers(REG_POS_DONE, 1)
        val = done[0] if done else None
        if val == idx:
            log(f"  -> arrived at stop {idx}"
                + ("  (UNVERIFIED: D903 already held this value)" if stale else ""))
            return
        if time.time() - t0 > timeout_s:
            raise TimeoutError(
                f"stop {idx} not reached within {timeout_s}s (D902={idx} was accepted, "
                f"D903={val}). The command register holds the right value, so the PLC "
                f"is not acting on it. Check on the HMI/AutoShop -- none of these are "
                f"visible over Modbus: AUTO mode selected (bMode_Auto), axis homed and "
                f"at standstill (bAxSt_StandStill), no servo/axis error, and bTestAbs "
                f"NOT stuck TRUE (it is ORed into the same bAx_AutoStrat timer, so it "
                f"suppresses the rising edge MC_MoveAbsolute needs).")
        await asyncio.sleep(poll_s)


def signal_capture_done(plc):
    """Optional handshake: tell the PLC the photo is done (D905 = 1)."""
    try:
        plc.client.write_single_register(REG_CAM_DONE, 1)
    except Exception as e:      # non-fatal in calibration
        log(f"  (capture-done handshake failed, ignoring: {e})")


async def run_capture(calib_root, n_stops, n_passes, start_pos=None, end_pos=None,
                      interactive=True, dry_run=False):
    from backend.plc_backend.async_plc_client import AsyncPLCClient  # noqa: E402

    calib_root = Path(calib_root)
    calib_root.mkdir(parents=True, exist_ok=True)

    # nominal per-stop rail position (mm), if the HMI start/end were provided.
    # Spacing divides by (n_stops - 1), NOT n_stops, so that stop 1 sits on
    # rStratPos and stop n_stops sits exactly on rEndPos -- matching the PLC's
    # rAx_TagPos := rStratPos + (iUp_PosNum-1)*rCameraDis. This is what V1.1's
    # ladder did (SUB UI_CameraNum K1 -> divide by N-1); V1.2's ST rewrite
    # regressed it to /N, which parks the last stop 1/N short of rEndPos.
    stop_x = None
    if start_pos is not None and end_pos is not None:
        if n_stops < 2:
            raise ValueError("--n-stops must be >= 2 to compute a stop spacing")
        cam_dis = (end_pos - start_pos) / (n_stops - 1)   # matches PLC rCameraDis
        stop_x = {f"{n:02d}": start_pos + (n - 1) * cam_dis for n in range(1, n_stops + 1)}
        log(f"nominal stop positions (mm): {stop_x}")

    rvc = None
    if dry_run:
        log("DRY RUN: skipping camera init/capture — gantry motion + register readback only.")
    else:
        log("initializing cameras ...")
        from backend.rvc_cameras.async_rvc import AsyncRVCXCameras  # noqa: E402
        rvc = AsyncRVCXCameras()
        if rvc.system_init() == 1:
            log("ERROR: camera init failed (check SNs in async_rvc.CAM_SN_DICT and GigE link).")
            return 1

    log("connecting to PLC ...")
    plc = AsyncPLCClient()
    # NOTE: intentionally NOT calling plc.reset() — on V1.2 that writes D902=1
    # (a spurious move command), not a reset.

    # set the stop count on the PLC (iCameraNum @ D906); PLC recomputes rCameraDis.
    plc.client.write_single_register(REG_CAMERA_NUM, n_stops)
    await asyncio.sleep(0.2)
    rb = plc.client.read_holding_registers(REG_CAMERA_NUM, 1)
    rb = rb[0] if rb else None
    if rb != n_stops:
        log(f"WARNING: iCameraNum readback D906={rb} != {n_stops} — write may have "
            f"failed; the gantry may not move (bData_Ok needs iCameraNum>0).")
    else:
        log(f"set iCameraNum (D906) = {n_stops}")

    try:
        for p in range(n_passes):
            pass_name = f"pass_{p:02d}"
            pass_dir = calib_root / pass_name
            if interactive and not dry_run:
                input(f"\n=== PASS {p}/{n_passes-1} ===\n"
                      f"Place / re-pose the CCT boards for this pass, then press Enter ...")
            if rvc:
                rvc.set_save_path(pass_dir)   # capture_dual writes <save_path>/NN/{left,right}
            log(f"{pass_name}: {'moving through' if dry_run else 'capturing'} "
                f"{n_stops} stops -> {pass_dir}")
            for n in range(1, n_stops + 1):
                await move_to_stop(plc, n)
                if dry_run:
                    log(f"  {pass_name}/{n:02d}: arrived (dry-run, no capture)")
                    continue
                await rvc.capture_dual(n)             # writes pass_dir/NN/{left,right}/
                signal_capture_done(plc)
                log(f"  {pass_name}/{n:02d} captured")
        log("dry run complete." if dry_run else "all passes captured.")
    finally:
        if rvc:
            try:
                rvc.system_shutdown()
            except Exception as e:
                log(f"camera shutdown warning: {e}")

    # write a manifest the calibrate_rig config can copy stop_x from
    manifest = {
        "calib_root": str(calib_root),
        "n_stops": n_stops,
        "n_passes": n_passes,
        "stop_names": [f"{n:02d}" for n in range(1, n_stops + 1)],
        "stop_x": stop_x,
        "note": "stop_x is nominal (rStratPos+(n-1)*rCameraDis, rCameraDis="
                "(rEndPos-rStratPos)/(iCameraNum-1)); operational_stops for an "
                "N-stop inspection = start+(k-1)*(end-start)/(N-1), k=1..N, "
                "so stop 1 is at rStratPos and stop N is at rEndPos",
    }
    with open(calib_root / "capture_manifest.json", "w", encoding="utf-8") as f:
        json.dump(manifest, f, indent=2, ensure_ascii=False)
    log(f"wrote {calib_root / 'capture_manifest.json'}")
    return 0


def main():
    ap = argparse.ArgumentParser(description="Multi-pass gantry calibration capture.")
    ap.add_argument("--calib-root", required=True, help="output root (pass-major)")
    ap.add_argument("--n-stops", type=int, required=True,
                    help="stops per pass; written to the PLC as iCameraNum (D906).")
    ap.add_argument("--n-passes", type=int, default=3, help="number of board-repose passes")
    ap.add_argument("--start-pos", type=float, default=None,
                    help="HMI rStratPos (mm), to log nominal stop positions")
    ap.add_argument("--end-pos", type=float, default=None,
                    help="HMI rEndPos (mm), to log nominal stop positions")
    ap.add_argument("--no-interactive", action="store_true",
                    help="skip the between-pass operator prompt (e.g. single pass)")
    ap.add_argument("--dry-run", action="store_true",
                    help="move through the stops + read back registers, but do NOT init or "
                         "fire the cameras (validates gantry + Modbus without PyRVC)")
    args = ap.parse_args()

    rc = asyncio.run(run_capture(
        args.calib_root, args.n_stops, args.n_passes,
        start_pos=args.start_pos, end_pos=args.end_pos,
        interactive=not args.no_interactive, dry_run=args.dry_run))
    sys.exit(rc or 0)


if __name__ == "__main__":
    main()
