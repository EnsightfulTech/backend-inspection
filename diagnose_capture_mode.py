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
import time

try:
    import PyRVC as RVC
except ImportError:
    print("[FAIL] Could not 'import PyRVC'.")
    sys.exit(1)

EXPECTED_SN = ["M2GM250B673", "M2GM250B674"]

# `--probe` additionally attempts a 2D-only capture and then a full 3D capture,
# timing both. Default stays read-only (settings dump, no capture).
PROBE = "--probe" in sys.argv

MODE_NAMES = {}
for name in ("CaptureMode_Normal", "CaptureMode_Fast", "CaptureMode_Ultra",
             "CaptureMode_AntiInterReflection", "CaptureMode_SwingLineScan",
             "CaptureMode_FixedLineScan"):
    if hasattr(RVC, name):
        MODE_NAMES[getattr(RVC, name)] = name


def dump_one(sn, device):
    print(f"\n=== {sn} ===")

    # Every SDK example checks this BEFORE capturing. A mismatch means the
    # camera firmware is not the version this SDK build expects -- upgrade via
    # RVCManager. RVCManager itself can keep working while an SDK capture fails.
    try:
        match = device.IsFirmwareMatch()
        print(f"  IsFirmwareMatch           = {match}"
              f"{'' if match else '   <-- MISMATCH: upgrade firmware in RVCManager'}")
    except Exception as e:
        print(f"  IsFirmwareMatch           = <error: {e}>")

    try:
        _, info = device.GetDeviceInfo()
        for attr in ("name", "sn", "type", "support_extra", "firmware_version",
                     "cameras_serial_number", "ip"):
            if hasattr(info, attr):
                print(f"  info.{attr:<21s} = {getattr(info, attr)}")
    except Exception as e:
        print(f"  GetDeviceInfo             = <error: {e}>")

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

    NA = "<not available in this PyRVC build>"
    fields = [
        "scan_times", "exposure_time_2d", "gain_2d", "gamma_2d",
        "exposure_time_3d", "gain_3d", "gamma_3d",
        "light_contrast_threshold", "projector_brightness",
        "hdr_exposure_times",
        "line_scanner_scan_time_ms", "line_scanner_exposure_time_us",
        "line_scanner_min_distance", "line_scanner_max_distance",
        "line_scanner_brightness_threshold", "line_scanner_laser_position",
        "line_scanner_confidence", "correspond2d", "trigger_mode",
        "use_auto_noise_removal", "confidence_threshold",
        "pointcloud_completion",
    ]
    for f in fields:
        print(f"  {f:<26s} = {getattr(opt, f, NA)}")

    hdr_n = getattr(opt, "hdr_exposure_times", 0) or 0
    for i in range(int(hdr_n)):
        try:
            print(f"  hdr[{i}] exposure/gain/brightness/scan_times = "
                  f"{opt.GetHDRExposureTimeContent(i)} / "
                  f"{opt.GetHDRGainContent(i)} / "
                  f"{opt.GetHDRProjectorBrightnessContent(i)} / "
                  f"{opt.GetHDRScanTimesContent(i)}")
        except Exception as e:
            print(f"  hdr[{i}]: <error reading: {e}>")

    if PROBE:
        print("  -- probe --")
        try:
            print(f"  GetBandwidth              = {x.GetBandwidth()}")
        except Exception as e:
            print(f"  GetBandwidth              = <error: {e}>")

        # 2D-only: one frame, no structured-light sequence, tiny data volume
        # next to a full Ultra 3D capture. If 2D succeeds and 3D times out,
        # the problem is the 3D acquisition/transfer, not the link itself.
        _timed(lambda: x.Capture2D(RVC.CameraID_Left), "Capture2D")
        _timed(lambda: x.Capture(opt), "Capture(3D)")

    x.Close()
    RVC.X2.Destroy(x)


def _timed(fn, label):
    """Run a capture call, reporting how long it took and the error code."""
    t0 = time.time()
    try:
        ok = fn()
    except Exception as e:
        print(f"  {label:<26s} = <exception after {time.time()-t0:.2f}s: {e}>")
        return
    dt = time.time() - t0
    if ok:
        print(f"  {label:<26s} = OK in {dt:.2f}s")
    else:
        print(f"  {label:<26s} = FAILED in {dt:.2f}s  "
              f"code={RVC.GetLastError()} msg={RVC.GetLastErrorMessage()}")


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
