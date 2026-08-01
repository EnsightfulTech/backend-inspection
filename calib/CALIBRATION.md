# Rig calibration — dual-camera + multi-stop, multi-pass

Joint extrinsic calibration for the wall-panel inspection rig: two RVC M52000
cameras side-by-side on a gantry that stops at 7 positions along a ~6 m rail.

One bundle adjustment, pooled over multiple capture passes, solves:

- **`T_lr`** — the left-right camera extrinsic (right-cam → left-cam), shared by all stops.
- **`T_s`** — every stop's trajectory pose (stop-s left-cam → a common reference frame),
  with the reference stop fixed as identity.

It then fits each pose degree-of-freedom as a smooth **`pose(x)`** curve vs rail position
(capturing the rail's mid-span **sag**), evaluates that model at the operational stops, and
writes the extrinsics in the exact format the inspection runtime consumes.

## Why this exists (vs. the old `optimize.py`)

- The old `generate_pkl_by_using_oneRT` propagated a *single constant* step transform, i.e.
  it assumed every inter-stop move is identical. The rail sags in mid-span, so per-stop tilt
  is **not** constant — that bakes in and accumulates drift. Here each stop is a free pose
  tied together by loop-closure markers and solved **jointly**, and the sag is modeled.
- The old Nelder-Mead solve reconstructed translation as `t = dt + u1 - u0`, dropping the
  `rot @ u0` term — a large error at 2.85 m standoff. `rig_geometry.umeyama` fixes this with
  a closed-form SVD rigid fit (`t = u1 - R @ u0`).
- Multiple passes (boards re-posed between passes) are pooled — the "moved chessboard, fixed
  rig" multi-pose trick — which averages out depth noise and breaks the coplanar degeneracy.

## Files

| file | role |
|------|------|
| `capture_calib.py`   | **online** capture driver: moves the gantry + fires both cameras into the pass-major tree. Runs on the rig PC (needs PyRVC + PLC). |
| `rig_geometry.py`    | pure numpy/scipy math: `umeyama`, pose ↔ 6-vec, `PoseXModel` (sag fit). Unit-testable. |
| `calibrate_rig.py`   | **offline** pipeline + CLI. `bundle_adjust` core is pure-numpy; camera-stack imports are lazy. |
| `test_calibrate_rig.py` | synthetic verification (no camera stack needed). |
| `requirements-calib.lock.txt` | verified pinned Python-3.10 env (frozen) for reproducible deploy. |
| `CALIBRATION.md`     | this document. |

These scripts live **inside** backend-inspection at `calib/` and import the repo directly
(`from algorithms.my_pcd import MyPCD`, `from backend.plc_backend...`); each adds the repo root
to `sys.path` at import so it runs both as `python calib/<script>.py` and `python -m calib.<script>`.
Camera I/O reuses `algorithms.my_pcd.MyPCD`, `algorithms.CCTDecoder.cct_decode.CCT_extract`,
`algorithms.utils.filter_avg` — no duplicated copies.

## Environment

**Use the existing `wallInspect` conda env** — the backend-inspection runtime (Python 3.10.20 x64).
It already has every dependency these scripts need, including a **PyRVC** binding; the calibration
test passes in it as-is, nothing to add.

The env is pinned to **Python 3.10 x64** on purpose: the PyRVC `.pyd` is ABI-locked to one Python
version and `open3d` needs `numpy < 2`. Do **not** use base 3.13.

```bash
# preferred: just use the existing wallInspect env (has everything, incl. PyRVC)
conda activate wallInspect
# to recreate elsewhere from scratch:
conda create -n wallInspect python=3.10 -y && conda activate wallInspect
pip install -r calib/requirements-calib.lock.txt   # exact versions verified 2026-07-31
pip install <pyrvc wheel>                           # see the PyRVC version note below
```

`calib/requirements-calib.lock.txt` is the frozen calib subset (73 packages); the full backend
`requirements.txt` is a superset, so an existing `wallInspect` needs nothing added. Also required
(not pip packages): this **backend-inspection checkout** (scripts import `algorithms.*` /
`backend.*`) and the **RVC SDK runtime** (`C:\Program Files\RVBUST`) for real capture.

### ⚠️ Windows Firewall must allow the Python interpreter (GigE stream channel)

**This is the single most likely reason capture fails on a freshly set-up PC.** Symptom
(observed 2026-08-01 during rig bring-up):

- `SystemListDevices` finds both cameras, `Open()` succeeds, `LoadCaptureOptionParameters()`
  returns correct settings, `IsFirmwareMatch()` is `True` — **everything on the control channel
  works**;
- but *every* capture — `Capture()` (3D) **and** `Capture2D()` — fails after a fixed ~6.1–6.3 s with
  **error 215 = 相机拍照超时 / capture timeout** (`RVCSDK\docs\ErrorCode.csv`), identically on both
  cameras;
- meanwhile **RVCManager captures the same cameras from the same PC in 3–4 s.**

Cause: GigE Vision cameras push image data to the host as *unsolicited inbound UDP*. Control
traffic is request/response and passes the firewall fine, so enumerate/open/read-settings all
succeed; the image stream is dropped, so the SDK waits out its internal timeout and reports 215.
RVCManager works because its installer registers a firewall exception — a conda `python.exe` has
none.

**Current workaround on the Guangzhou rig PC: Windows Firewall is switched off.** That machine is
not normally connected to the public internet, so this was an accepted operational trade-off. With
the firewall off, capture works immediately — expected timings (RVC-M52000, `CaptureMode_Ultra`,
HDR off): `Capture2D` ≈ 0.15 s, `Capture` (3D) ≈ 2.4 s, image 2448×2048.

**Allow rules did *not* work — do not assume they will.** All of the following were tried on the
rig PC (2026-08-01) and capture still timed out with 215 until the firewall was turned off entirely:

- a per-program inbound allow rule for the env's `python.exe` (`netsh advfirewall firewall add rule
  name="PyRVC python" dir=in action=allow program="...\python.exe" enable=yes profile=any`) —
  verified present, enabled, inbound, allow, all profiles, any protocol;
- checking for conflicting inbound **Block** rules for any `python.exe` (block wins over allow);
- an IP-scoped rule allowing inbound UDP from the two camera addresses regardless of program.

A plausible explanation is that RVBUST bundles Hikvision MVS components (`MVGigEVisionSDK.dll`,
`MvProducerGEV.cti`) and some GigE vendors ship a filter/performance driver that receives stream
packets below the normal Windows socket layer, where per-application firewall rules don't apply —
but this was **not** confirmed. Unresolved; worth raising with RVBUST if a firewall-on deployment
is ever required.

If you do need the firewall on elsewhere, note it can be disabled per profile rather than globally
(`Get-NetConnectionProfile` to find the camera NIC's profile, then
`Set-NetFirewallProfile -Profile <name> -Enabled False`), which leaves the other profiles active.

Also note, if allow rules are ever revisited: such a rule is **per executable**, and
`run_backend.bat` launches a *hardcoded* python.exe path — a rule covering the interpreter used for
testing will not necessarily cover the one that runs `main.py` in production.

Ruled out before finding this, recorded so it isn't re-investigated: capture mode (fails
identically on SwingLineScan / Normal / Ultra — note the message text says "Normal Collect Failed"
even when the mode is Ultra, so that word is the collection routine, not the mode), HDR bracketing,
device-occupied (RVCManager fully closed — otherwise the camera won't even open), DLL path
conflicts (only one `RVC.dll`/`RVC_C.dll` on `PATH`), firmware mismatch, trigger mode
(`TriggerMode_SoftWare`, correct), and PyRVC version.

### PyRVC — version note (updated 2026-08-01)

Either binding works; **the firewall above was the real blocker**, and it affected 1.14.0 and
1.15.0 identically.

- **1.14.0** — `pip install PyRVC==1.14.0`, straight from PyPI, no toolchain needed. Does **not**
  expose `trigger_mode` or `line_scanner_confidence` on `X2_CaptureOptions`, nor `GetBandwidth`.
- **1.15.0** — built from the on-disk SDK source (supplier's recommendation); exposes those extra
  fields, which is how the trigger configuration was verified. Deployed and confirmed capturing on
  the rig. Prebuilt wheel:

  ```
  C:\Users\yuany\rvc_build\dist\pyrvc-1.15.0-cp310-cp310-win_amd64.whl   (49.3 MB)
  ```

The wheel is portable — see the build recipe below; the rig PC needs no C++ toolchain.

#### Building the 1.15.0 wheel (only needed once, on a machine with MSVC)

The wheel is portable: **build once, copy the `.whl` to the rig PC and `pip install` it there** —
no Visual Studio / CMake / `RVC_ROOT` / SDK source needed on the target. The only requirements on
the target are Python **3.10 x64** (the `cp310` tag) and a VC++ 2015–2022 x64 redistributable
(install that only if `import PyRVC` fails with `DLL load failed`). The wheel bundles the whole RVC
runtime (~49 MB of DLLs) into site-packages, but the target still needs the **full RVC SDK
installed separately** for GigE drivers / RVCManager / firmware.

```bash
# 1. copy the SDK out of Program Files — setup.py copies runtime DLLs into its own
#    directory first, which fails with PermissionError under Program Files
xcopy /E /I "C:\Program Files\RVBUST\RVC\RVCSDK" "C:\Users\yuany\rvc_build\RVCSDK"
# 2. CMakeLists.txt reads RVC_ROOT for include/ and lib/
set RVC_ROOT=C:\Users\yuany\rvc_build\RVCSDK
# 3. CMake 4.x removed compatibility with the SDK's cmake_minimum_required(VERSION 2.8.12)
set CMAKE_POLICY_VERSION_MINIMUM=3.5
# 4. --no-build-isolation: pip's isolated build env cannot resolve a pip-installed cmake
#    (the launcher runs but 'No module named cmake'), even though `cmake --version` works
cd /d C:\Users\yuany\rvc_build\RVCSDK\PyRVC
pip wheel --no-build-isolation --no-deps -w C:\Users\yuany\rvc_build\dist .
```

Do **not** add the wheel to git — 49 MB per clone is a real cost, especially over a
mainland-China mirror. Keep it with the deployment artifacts.

#### Troubleshooting helpers (repo root, standalone — no PLC/CloudComPy)
- `diagnose_capture_mode.py` — dumps a camera's stored `X2_CaptureOptions` (capture mode, HDR
  brackets, line-scan params) without capturing. Fields missing from a given build print as
  `<not available in this PyRVC build>`.
- `test_camera_connection.py` — enumerates, opens, and captures one frame per camera. Reports the
  numeric `GetLastError()` code (look it up in `RVCSDK\docs\ErrorCode.csv`), and tests both cameras
  independently so one failure doesn't mask the other's result.

### What you can verify on the hardware-less sandbox
- `pip install -r calib/requirements-calib.lock.txt` resolves cleanly (env is correct).
- `python calib/test_calibrate_rig.py` passes (solver + geometry, no hardware).
- `python -c "import PyRVC"` succeeds (binding + DLLs load; 0 cameras is fine).
- **Cannot** run `capture_calib.py` (even `--dry-run` needs the PLC reachable) or a real
  `calibrate_rig.py` (needs captured `.ply/.png`).

## Capture procedure

1. **Lay a fixed, continuous CCT field** spanning the ~6 m travel and leave it put for the
   whole pass. Recommended: the 16 boards (144 unique IDs) in a brick-staggered 2-row layout,
   with a few raised/tilted so the markers span a 3-D volume (needed for rotation/pitch
   observability). Route some markers through the central left-right overlap wedge (|y| ≲ 566 mm
   at floor) so the same field also constrains `T_lr`.
2. **Capture every stop** over that fixed field: at each of the 7 rail stops, grab both
   cameras (`Image.png` + `PointCloud.ply` per camera). Don't move the boards mid-pass.
3. **Repeat as multiple passes**, re-posing / re-tilting / nudging the boards between passes.
   3–5 passes is a good start. IDs may repeat across passes (the boards moved); the solver
   treats the same ID in a different pass as a *different* physical point.
4. *(Optional, recommended at 6 m)* tape/total-station a few marker-to-marker distances to
   anchor metric scale against long-range drift.

## Directory layout (pass-major)

```
calib_root/
  pass_00/
    stop_00/ left/{Image.png,PointCloud.ply}   right/{Image.png,PointCloud.ply}
    stop_01/ ...
    ...
    stop_06/
  pass_01/            # boards re-posed
    stop_00/ ...
  pass_02/ ...
```

## Config

JSON file passed via `--config` (CLI flags `--calib-root/--out-dir/--reference-stop` override):

```json
{
  "calib_root": "D:/CalibData/2026-07-29",
  "out_dir":    "D:/CalibData/2026-07-29/calib_out",
  "reference_stop": "stop_00",
  "cct_n": 12,
  "stop_x": { "stop_00": 0, "stop_01": 1000, "stop_02": 2000, "stop_03": 3000,
              "stop_04": 4000, "stop_05": 5000, "stop_06": 6000 },
  "operational_stops": { "01": 0, "02": 1000, "03": 2000, "04": 3000,
                         "05": 4000, "06": 5000, "07": 6000 },
  "model": { "kind": "poly", "poly_deg": 3 },
  "crosscheck_tol_mm": 2.0,
  "crosscheck_tol_deg": 0.05
}
```

- **`stop_x`** — rail position (mm) of each *captured* stop. Drives the sag curve's x-axis.
  If omitted, ordinal positions are used and the sag axis is in "stop units" (warns).
- **`operational_stops`** — `{output_key: rail_x_mm}`. `pose(x)` is evaluated at each `rail_x_mm`
  and written under `output_key`. **The keys must equal the folder names the inspection capture
  writes** — currently `"01".."08"` (`async_rvc.capture_dual` uses `str(idx).zfill(2)`), which is
  how `combine_frames_extrinsic` looks up `traj_ext[idx]`.
- **`model.kind`** — `"poly"` (default, degree `poly_deg`; matches a beam-deflection shape) or
  `"spline"` (smoothing spline) when you have many samples.

## Running — step 1: capture (online, on the rig PC)

`capture_calib.py` drives the gantry to each stop and fires both cameras into the pass-major
tree, prompting you to re-pose the boards between passes.

```bash
python calib/capture_calib.py --calib-root D:/CalibData/2026-07-29 \
    --n-stops 21 --n-passes 3 --start-pos <rStratPos_mm> --end-pos <rEndPos_mm>
```

- The script **writes `iCameraNum` over Modbus (D906)** from `--n-stops`. On the **HMI** you
  still set a valid **travel range** (`rStratPos < rEndPos`, `rStratPos > 0`), then select
  **AUTO mode** and home the axis. Together these satisfy `bData_Ok`. If the gantry doesn't
  move, check AUTO mode / homing / travel range first.
- `--start-pos/--end-pos` are only used to log *nominal* stop positions into
  `capture_manifest.json` (copy them into the offline config's `stop_x`). The rail's actual
  position (`rAx_RealPos`) is not Modbus-exposed in V1.2.
- Uses a **corrected move-and-wait** (waits for `D903 == commanded index`); it deliberately does
  **not** call the backend `AsyncPLCClient.reset()` (on V1.2 that writes `D902=1`, a spurious
  move) nor rely on `wait_for_inpos()` (breaks on `D903 != 0`, which is stale after the first
  stop). Your production `async_plc_client.py` is left untouched.

**PLC V1.2 register map** (confirmed from the variable table, and verified live on the rig
2026-08-01 — reads, and a `set_camera_num(7)` write, both round-trip correctly):
`D902 iUp_PosNum` (write index), `D903 iUp_PosNum_Last` (reads back the index when `bAx_AbsDone`),
`D904 iProdrdy_ToUp` (wall in position, PLC→host), `D905 iUp_CameraOk` (capture done),
`D906 iCameraNum` (write stop count — PLC derives `rCameraDis` from it). `rStratPos/rEndPos`
remain HMI-only.

`D906` is **non-retentive**: it resets to 0 on every PLC power cycle, and while it is 0 the PLC's
`bData_Ok` guard is false so it ignores every position command. `HardwareManager` now writes
`config.NUM_CAPTURE_POSITIONS` into it at startup for this reason.

### ⚠️ PLC change required — stop spacing divisor

`MAIN.ST` currently computes, **unguarded, every scan**:

```pascal
rCameraDis := (rEndPos-rStratPos)/INT_TO_REAL(iCameraNum);      -- wrong divisor
```

Two problems:

1. **Wrong divisor.** With `rAx_TagPos := rStratPos + (iUp_PosNum-1)*rCameraDis`, dividing by `N`
   puts the last stop at `rStratPos + (N-1)/N·range` — 1/N short of `rEndPos`, so the far end of
   the wall is never photographed. V1.1 divided by `N-1`; the V1.2 ST rewrite regressed it.
2. **Division by zero.** It is not inside the `bData_Ok` guard, so while `iCameraNum = 0` (its
   power-up value) the PLC divides by zero on every scan, and `iPoint_AutoByKey` then divides by
   that result again.

Corrected form:

```pascal
IF iCameraNum > 1 THEN
    rCameraDis := (rEndPos-rStratPos)/INT_TO_REAL(iCameraNum-1);
END_IF
```

Also worth doing at the same time: initialise `iCameraNum := k7;` in the existing `IF M8002 THEN`
power-up block (alongside `rAx_AbsAcc` etc.) so the PLC always powers into a usable state, and
widen `bData_Ok`'s `iCameraNum>0` test to `iCameraNum>1`, since `N=1` now divides by zero.

`capture_calib.py` and this document already use the corrected `N-1` form, so they will only agree
with the rig **after** the PLC is updated.

**Status 2026-08-01: all three of the above are done and downloaded** — the monitor shows
`rStratPos=100, rEndPos=5500, iCameraNum=7 → rCameraDis=900.0`, and a 7-stop dry run lands stop 7
on `rEndPos`.

### ⚠️ PLC: latched `bAx_AbsStrat_M3` (fixed 2026-08-01)

`bAx_AbsStrat_M3` was set inside the auto-positioning `IF` with **no `ELSE`**, so once true it
stayed true and `MC_MoveAbsolute`'s `Execute` never saw a second rising edge — only the first move
of a session ran. Fixed by making it self-clearing, like its `M1`/`M2` siblings:

```pascal
    bAx_AbsStrat_M3 := TRUE;
ELSE
    bAx_AbsStrat_M3 := FALSE;
END_IF
```

### ⚠️ PLC: `bAx_AutoStrat` off-delay swallows the next rising edge — STILL OPEN

`MC_MoveAbsolute` triggers on a **rising edge** of `Execute`, driven from an off-delay timer whose
reset is not wired:

```pascal
TOFR(IN := (M1 or M2 or M3 or bTestAbs) and bAxSt_StandStill and bMode_Auto,
     PT := k100, R := , Q => bAx_AutoStrat);
```

When a move completes, `M3` drops (`iUp_PosNum == iUp_PosNum_Last`) but `Q` stays HIGH for
`PT = 100 ms`. A host that polls `D903` and commands the next stop within that window — tens of
milliseconds, so always — raises `M3` again before `Q` falls. No falling edge means no rising edge,
`MC_MoveAbsolute` never re-triggers, and the axis **deadlocks with `Execute` stuck ON**, since `IN`
then never goes false again.

Symptom: stop 1 always succeeds (`Q` starts low after an idle period or an AUTO-mode toggle), every
later stop hangs, with the monitor showing a correct `rAx_TagPos`, `M3` ON, `bAx_AutoStrat` ON, axis
at standstill, and **no axis or servo error**.

Fix — wire the reset, exactly as the commented-out `M1` rung in `MAIN.ST` already does:

```pascal
TOFR(IN := (...) AND bAxSt_StandStill AND bMode_Auto,
     PT := k100, R := bAx_AbsDone, Q => bAx_AutoStrat, ET => );
```

Until that is downloaded, the host works around it by pausing `POST_ARRIVAL_SETTLE_S` (0.3 s) after
each arrival — see `capture_calib.py` and `backend/plc_backend/async_plc_client.py`. Keeping that
pause after the PLC is fixed is cheap (≈2 s per 7-stop sweep) and guards against the same class of
edge-timing bug.

### ⚠️ PLC: `MC_MoveAbsolute` error outputs are unwired

In `SBR_AxisDeal.ST`, `Error`, `ErrorID` and `Busy` are all left blank on `MC_MoveAbsolute`. Any
rejection of a move is therefore completely silent — nothing to read, and `iAx_AxErID`/`iAx_SrvErID`
stay 0 because a function-block error is not an axis fault. That is what made the off-delay bug
above take several test rounds to isolate. Wire them:

```pascal
                Busy => bAx_AbsBusy,
                Error => bAx_AbsEr,
                ErrorID => iAx_AbsErID);
```

### Gotcha: `iUp_PosNum_Last` (D903) is not cleared between runs

Only an AUTO-mode rising edge zeroes it. So a fresh run can start with `D903` already equal to the
first stop index, which makes that command a no-op (the PLC acts only on
`iUp_PosNum <> iUp_PosNum_Last`) and any naive "wait for D903 == idx" pass instantly having moved
nothing. `capture_calib.py` detects and reports this rather than reporting a false arrival — but it
means **stop 1 of a sweep may legitimately not move**. Toggle the mode switch to AUTO before a real
capture to start from a clean state.

## Running — step 2: calibrate (offline)

Needs an env with **open3d, opencv, scipy, loguru** (the backend runtime env) plus matplotlib
for the plot. It reads saved `.ply/.png` — **`PyRVC` is not needed**.

```bash
python calib/calibrate_rig.py --config calib_config.json
```

Set the config's `operational_stops` x-values from the **operational** PLC formula
`x_k = rStratPos + (k-1)·(rEndPos - rStratPos)/(N-1)`, k=1..N (N = 7 here, so the divisor is
**6**) — the continuous `pose(x)` model interpolates the dense calibration grid to exactly those
positions, so the calibration stops need not coincide with the 7 operational stops.

Note the divisor is **N-1, not N**, so stop 1 lands on `rStratPos` and stop N lands exactly on
`rEndPos`. This matches the PLC's `rAx_TagPos := rStratPos + (iUp_PosNum-1)*rCameraDis` and what
V1.1's ladder computed (`SUB UI_CameraNum K1` before the divide). ⚠️ **V1.2's ST rewrite regressed
this to `/iCameraNum`**, which parks the last stop 1/N short of `rEndPos`; see "PLC change
required" below. An earlier draft of this document copied the V1.2 `/7` form — that was wrong.

## Outputs (in `out_dir`)

- **`left_right_ext.pkl`** — single 4×4 `T_lr` (right → left). Drop-in for `config.CAM_EXT_PKL`.
- **`cam_traj_ext.pkl`** — `{operational_key: 4×4}` (left-cam → reference). Drop-in for `config.TRAJ_EXT_PKL`.
- **`calib_report.json`** — overall / per-stop / per-pass / L-R residual RMS (mm), all poses,
  and the pose(x) cross-check.
- **`sag_curve.png`** — 6 panels (rotvec x/y/z, t x/y/z) vs rail x: BA per-stop points + fitted
  pose(x) curve.

### Reading the sag plot

Each panel is one DOF of the stop pose vs rail position. A smooth **bow peaking near mid-span**
is the expected rail deflection (largest in `rotvec_y`/pitch and `t_z`/droop). The blue dots are
the independent BA estimates; the orange curve is the fitted model deployed to the operational
stops. Dots scattering well off a smooth curve, or a kink at a stop, flags a bad board/stop or a
non-smooth rail (joint/support) — investigate before trusting interpolation there. The JSON
cross-check warns automatically when the model and the direct estimate disagree beyond tolerance.

## Integration with inspection

Point `config.py` at the new pickles and inspection fuses with the calibrated rig:

```python
CAM_EXT_PKL  = r"...\calib_out\left_right_ext.pkl"
TRAJ_EXT_PKL = r"...\calib_out\cam_traj_ext.pkl"
```

`algorithms/calib_concant.py::combine_frames_extrinsic` applies them per stop as
`left.transform(traj_ext[idx])` and `right.transform(traj_ext[idx] @ cam_ext)`.

## Verification

```bash
python calib/test_calibrate_rig.py    # numpy/scipy only; no camera stack
```

Runs `rig_geometry` unit checks and a synthetic end-to-end (known `T_lr` + known `pose(x)` sag,
noisy projected marker field, multi-pass) asserting recovery within noise. Reference result on
0.4 mm depth noise: `T_lr` err ≈ 0.004°/0.08 mm, per-stop ≤ 0.03°/1 mm, residual RMS ≈ 0.55 mm.

**Still to do on real hardware** (needs the capture data + backend env, cannot be done offline):
run on one real pass and confirm the residual RMS is mm-scale and the `cam_traj_ext.pkl` keys
match the inspection capture folder names; then feed both pickles to `combine_frames_extrinsic`
and visually confirm the L-R and inter-stop clouds overlap with no ghosting.

## Assumptions

- Rigid crossbeam ⇒ `T_lr` is one matrix valid at every stop. If you suspect the beam itself
  flexes with rail position, calibrate `T_lr` at an end stop and again at mid-span and compare.
- Gantry stop repeatability is validated, so the calibrated per-stop poses are reusable for
  inspection (the calibration reuses the same 7 stops).
- The 3-D point per marker comes from each camera's own point cloud (depth-noise-limited);
  multi-pass pooling is what drives that noise down.
