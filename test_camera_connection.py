"""
Standalone RVC-X camera connection test.

Purpose: verify the PC actually sees BOTH cameras over GigE, that their serial
numbers match what the backend expects, that each opens, and that each can grab
one frame. Does NOT touch the PLC, the websocket server, or CloudComPy.

Run on the device with both cameras connected:
    python test_camera_connection.py

Requires the PyRVC SDK to be installed in the active Python environment
(it is intentionally NOT in requirements.txt).
"""

import sys
from pathlib import Path

try:
    import PyRVC as RVC
except ImportError:
    print("[FAIL] Could not 'import PyRVC'. Install the RVC-X SDK in this "
          "environment first (it is not covered by requirements.txt).")
    sys.exit(1)

import numpy as np
import cv2

# These must match the two physical cameras. Same values as async_rvc.py.
EXPECTED_SN = ["M2GM250B673", "M2GM250B674"]
SAVE_DIR = Path("test_capture")


def main():
    RVC.SystemInit()
    try:
        # 1) Enumerate everything on the wire.
        ret, devices = RVC.SystemListDevices(RVC.SystemListDeviceTypeEnum.GigE)
        print(f"[INFO] GigE devices found: {len(devices)}")
        if len(devices) == 0:
            print("[FAIL] No cameras enumerated. Check power, cabling, and that "
                  "the NIC + cameras are on the same subnet (jumbo frames on).")
            return 1

        # 2) Print every serial number we can see.
        seen = {}
        for d in devices:
            _, info = d.GetDeviceInfo()
            print(f"       SN={info.sn}")
            seen[info.sn] = d

        # 3) Check the two we expect are present.
        missing = [sn for sn in EXPECTED_SN if sn not in seen]
        if missing:
            print(f"[FAIL] Expected serials not found: {missing}")
            print("       Update EXPECTED_SN here and CAM_SN_DICT in "
                  "backend/rvc_cameras/async_rvc.py to the SNs printed above.")
            return 1
        print(f"[ OK ] Both expected cameras present: {EXPECTED_SN}")

        # 4) Open each, capture one frame, save image + point cloud.
        # Each camera is independent -- one failing should not stop the other
        # from being tested, so failures are logged and looped past rather
        # than aborting the whole script.
        results = {}
        for role, sn in zip(("left", "right"), EXPECTED_SN):
            x = RVC.X2.Create(seen[sn])
            if not x.IsValid():
                print(f"[FAIL] {role} ({sn}): device handle not valid.")
                results[role] = False
                continue
            if not (x.Open() and x.IsOpen()):
                print(f"[FAIL] {role} ({sn}): failed to open.")
                RVC.X2.Destroy(x)
                results[role] = False
                continue
            print(f"[ OK ] {role} ({sn}): opened.")

            # Load the camera's stored options and pass them back explicitly.
            # Per RVC.h, bare Capture() also loads options from the camera, so
            # this is equivalent -- but being explicit means the settings
            # diagnose_capture_mode.py prints are provably the ones used here.
            # (The "X2 Normal Collect Failed!" text appears even when the mode
            # is Ultra, so that word is the collection routine, not the mode.)
            ret_opt, cap_opt = x.LoadCaptureOptionParameters()
            if not ret_opt:
                print(f"[FAIL] {role} ({sn}): LoadCaptureOptionParameters failed. "
                      f"{RVC.GetLastErrorMessage()}")
                x.Close(); RVC.X2.Destroy(x)
                results[role] = False
                continue

            if not x.Capture(cap_opt):
                print(f"[FAIL] {role} ({sn}): capture failed. "
                      f"code={RVC.GetLastError()} msg={RVC.GetLastErrorMessage()} "
                      f"-- look up 'code' in RVCSDK/docs/ErrorCode.csv")
                x.Close(); RVC.X2.Destroy(x)
                results[role] = False
                continue

            out = SAVE_DIR / role
            out.mkdir(parents=True, exist_ok=True)
            img = x.GetImage(RVC.CameraID_Left)
            w, h = img.GetSize().cols, img.GetSize().rows
            cv2.imwrite(str(out / "Image.png"), np.array(img, copy=False))
            pm = x.GetPointMap()
            pm.SaveWithImage(str(out / "PointCloud.ply"), img,
                             RVC.PointMapUnitEnum.Meter)
            print(f"[ OK ] {role} ({sn}): captured {w}x{h} -> {out}")
            results[role] = True

            x.Close()
            RVC.X2.Destroy(x)

        if all(results.values()):
            print("\n[DONE] Both cameras enumerated, opened, and captured. "
                  f"See '{SAVE_DIR}\\left' and '{SAVE_DIR}\\right'.")
            print("       Open the two Image.png to confirm left/right are not "
                  "swapped before trusting CAM_SN_DICT.")
            return 0
        else:
            failed = [role for role, ok in results.items() if not ok]
            print(f"\n[SUMMARY] failed: {failed}  ok: "
                  f"{[r for r, ok in results.items() if ok]}")
            return 1
    finally:
        RVC.SystemShutdown()


if __name__ == "__main__":
    sys.exit(main())
