"""
One-off diagnostic: read and print the RVC camera's currently stored capture
settings (whatever RVCManager or a previous script last configured), without
changing anything. Standalone — no PLC/websocket/CloudComPy/repo imports.

Purpose: `test_camera_connection.py`'s bare `x.Capture()` failed with
"X2 SwingLineScan Collect Failed!" — this prints out exactly what capture
mode + line-scan parameters are stored on the camera so we can tell whether
the mode itself is wrong for this application, or the mode is right but the
distance range doesn't match whatever the camera is currently pointed at.

Run on the rig PC:
    python diagnose_capture_mode.py
"""

import sys

try:
    import PyRVC as RVC
except ImportError:
    print("[FAIL] Could not 'import PyRVC'.")
    sys.exit(1)

EXPECTED_SN = ["M2GM250B673", "M2GM250B674"]

MODE_NAMES = {}
for name in ("CaptureMode_Normal", "CaptureMode_Fast", "CaptureMode_Ultra",
             "CaptureMode_AntiInterReflection", "CaptureMode_SwingLineScan",
             "CaptureMode_FixedLineScan"):
    if hasattr(RVC, name):
        MODE_NAMES[getattr(RVC, name)] = name


def dump_one(sn, device):
    print(f"\n=== {sn} ===")
    x = RVC.X2.Create(device)
    if not x.IsValid():
        print("  [FAIL] device handle not valid")
        return
    if not (x.Open() and x.IsOpen()):
        print("  [FAIL] failed to open")
        RVC.X2.Destroy(x)
        return

    ret, opt = x.LoadCaptureOptionParameters()
    if not ret:
        print("  [FAIL] LoadCaptureOptionParameters() failed:", RVC.GetLastErrorMessage())
        x.Close()
        RVC.X2.Destroy(x)
        return

    mode_name = MODE_NAMES.get(opt.capture_mode, f"<unknown:{opt.capture_mode}>")
    print(f"  capture_mode              = {mode_name}")
    print(f"  scan_times                = {opt.scan_times}")
    print(f"  exposure_time_3d          = {opt.exposure_time_3d}")
    print(f"  line_scanner_scan_time_ms = {opt.line_scanner_scan_time_ms}")
    print(f"  line_scanner_exposure_us  = {opt.line_scanner_exposure_time_us}")
    print(f"  line_scanner_min_distance = {opt.line_scanner_min_distance}")
    print(f"  line_scanner_max_distance = {opt.line_scanner_max_distance}")
    print(f"  line_scanner_confidence   = {opt.line_scanner_confidence}")
    print(f"  correspond2d              = {opt.correspond2d}")
    print(f"  trigger_mode              = {opt.trigger_mode}")

    x.Close()
    RVC.X2.Destroy(x)


def main():
    RVC.SystemInit()
    try:
        ret, devices = RVC.SystemListDevices(RVC.SystemListDeviceTypeEnum.GigE)
        if len(devices) == 0:
            print("[FAIL] No cameras enumerated.")
            return 1
        seen = {}
        for d in devices:
            _, info = d.GetDeviceInfo()
            seen[info.sn] = d
        for sn in EXPECTED_SN:
            if sn in seen:
                dump_one(sn, seen[sn])
            else:
                print(f"\n=== {sn} === [FAIL] not found on the wire")
        return 0
    finally:
        RVC.SystemShutdown()


if __name__ == "__main__":
    sys.exit(main())
