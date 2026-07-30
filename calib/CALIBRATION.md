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

**Use the existing `wallinspect` conda env** — the backend-inspection runtime (Python 3.10.20 x64).
It already has every dependency these scripts need, including a working **PyRVC 1.14.0**
(`PyRVC.cp310-win_amd64.pyd`); the calibration test passes in it as-is, nothing to add.

The env is pinned to **Python 3.10 x64** on purpose: the PyRVC `.pyd` is ABI-locked to one Python
version and `open3d` needs `numpy < 2`. Do **not** use base 3.13.

```bash
# preferred: just use the existing wallinspect env (has everything, incl. PyRVC 1.14.0)
conda activate wallinspect
# to recreate elsewhere from scratch:
conda create -n wallinspect python=3.10 -y && conda activate wallinspect
pip install -r calib/requirements-calib.lock.txt   # exact versions verified 2026-07-31
pip install PyRVC==1.14.0                           # the working binding (see note below)
```

`calib/requirements-calib.lock.txt` is the frozen calib subset (73 packages); the full backend
`requirements.txt` is a superset, so an existing `wallinspect` needs nothing added. Also required
(not pip packages): this **backend-inspection checkout** (scripts import `algorithms.*` /
`backend.*`) and the **RVC SDK runtime** (`C:\Program Files\RVBUST`) for real capture.

### PyRVC — version note
- The working, installed binding is **PyRVC 1.14.0** (`site-packages\PyRVC.cp310-win_amd64.pyd`),
  installed via `pip install PyRVC==1.14.0`. **Keep it.**
- The SDK **source** on disk is 1.15.0 but is **not prebuilt** — building it needs a full C++
  toolchain (CMake + MSVC + `RVC_ROOT` set + a writable build dir) and is unnecessary since 1.14.0
  imports and works. Left as-is by decision (2026-07-31).

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

**PLC V1.2 register map** (confirmed from the variable list): `D902 iUp_PosNum` (write index),
`D903 iUp_PosNum_Last` (reads back the index when `bAx_AbsDone`), `D905 iUp_Camera0k` (capture
done), `D906 iCameraNum` (write stop count — PLC derives `rCameraDis` from it). `rStratPos/rEndPos`
remain HMI-only.

## Running — step 2: calibrate (offline)

Needs an env with **open3d, opencv, scipy, loguru** (the backend runtime env) plus matplotlib
for the plot. It reads saved `.ply/.png` — **`PyRVC` is not needed**.

```bash
python calib/calibrate_rig.py --config calib_config.json
```

Set the config's `operational_stops` x-values from the **operational** PLC formula
`x_k = rStratPos + (k-1)·(rEndPos - rStratPos)/7`, k=1..7 — the continuous `pose(x)` model
interpolates the dense calibration grid to exactly those positions, so the calibration stops
need not coincide with the 7 operational stops.

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
