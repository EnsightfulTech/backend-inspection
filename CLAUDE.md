# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Backend server for an automated wall-panel (墙板) inspection system built for CSCEC. It drives a gantry-mounted
PLC + dual RVC-X stereo camera rig to capture a wall from 8 positions, fuses the resulting point clouds into one
combined cloud, aligns it against a CAD (DXF) reference for that wall model, computes dimensional deviations
(height/width/diagonal, window/door openings), and exports results as JSON/Excel/PDF. It talks to a separate
frontend project (`ensightful-control`, not in this repo) over a websocket + HTTP API on port 1337.

## Commands

There is no build system, linter, or pytest suite configured (no `pytest.ini`/`pyproject.toml`).

```bash
# install python deps (see "Environment gotchas" below for what's NOT covered by this)
pip install -r requirements.txt

# run the backend server (must run inside the CloudComPy310 environment, see below)
python main.py

# launch both frontend (npm run dev) and backend as subprocesses via the .bat files
python run.py

# standalone hardware-dependency check (no PLC/websocket/CloudComPy needed) —
# safe to run on a sandbox PC before cloning the env to the rig PC
python test_camera_connection.py
```

There is no `tests/` directory and no pytest suite — an earlier `tests/` folder of ad hoc, non-pytest debug
scripts (hardcoded personal paths, no assertions) was removed. [test_camera_connection.py](test_camera_connection.py)
at the repo root is the current pattern for a standalone hardware-check script: import only what it needs
(here, `PyRVC`), never touch `config.py`/`hardware_manager.py`, and fail with a clear message rather than
hanging if hardware/SDKs aren't present. Follow that pattern for any future one-off verification script instead
of reviving a `tests/` folder of unmaintained scripts.

## Environment gotchas (read before touching entrypoints)

- **CloudComPy must be activated before `import cloudComPy`.** [main.py](main.py) and
  [algorithms/measure_compare/measurement.py](algorithms/measure_compare/measurement.py) both `import cloudComPy`
  (CloudCompare's Python bindings, used for CAD-aligned point cloud preprocessing/slicing) — this only works if
  [envActivation.py](envActivation.py)'s `setup_environment()` has already patched `PYTHONPATH`/`PATH` to point at
  a local CloudComPy install. The install location is hardcoded: `C:\workspace\CloudComPy310\envCloudComPy.bat`.
  The project is meant to run inside a conda env literally named `CloudComPy310`.
- **`requirements.txt` does not cover everything.** `PyRVC` (proprietary RVC-X camera SDK) is listed only as a
  comment — install it separately. `cloudComPy` and `pywin32` (`win32com`, used for Excel→PDF export in
  `measurement.py`) aren't listed at all and must be present in the environment already.
- **Two independent PLC connections exist.** Gantry motion/position waiting uses Modbus TCP
  ([backend/plc_backend/async_plc_client.py](backend/plc_backend/async_plc_client.py), `config.PLC_HOST`/`PLC_PORT`).
  Reading the wall index/model off the line uses a separate Siemens S7 connection via `python-snap7`
  ([backend/plc_backend/snap7_client.py](backend/plc_backend/snap7_client.py), hardcoded IP `192.168.110.180`),
  gated by `config.GET_MODEL_FROM_PLC`. `snap7_client.py` opens its connection at import time — importing it
  without a reachable PLC will fail immediately (hence it's only imported when `RUN_SIMULATION` is `False`).
  The register/coil numbers used here (`PLC_D901`/`PLC_D904`/`PLC_D905`, DB 96 offsets) are only meaningful
  in light of the actual ladder logic — the Huichuan (汇川) PLC project ("中建桁架PLC_V1.1", `.LD` ladder files)
  lives outside this repo at `C:\Users\yuany\中建桁架PLC_V1.1` and is the authoritative source for that mapping.
- **Many paths are hardcoded per-machine** outside of `config.py`: `envActivation.py`'s CloudComPy path, and
  [run_backend.bat](run_backend.bat)/[run_frontend.bat](run_frontend.bat) (which `cd` into hardcoded
  `C:\workspace\backend_inspection` / `C:\workspace\ensightful-control` and invoke a hardcoded conda python).
  These need manual editing per deployment machine — they are not read from `config.py`.
- Windows-only: `windows_toasts` (desktop notifications on every websocket connect / hardware init, see
  [backend/utils.py](backend/utils.py)) and the Excel COM automation in `measurement.py`.

## Configuration (`config.py`)

Central place to change per-deployment behavior (per the project README):

- `RUN_SIMULATION` — if `True`, never touches PLC or RVC cameras; `HardwareManager` reads pre-captured frames
  from `SIMULATION_DATA_DIR` instead (folders `01`..`08`, each with `left/`/`right/` containing `Image.png`,
  `PointCloud.ply`, `Depth.tif`).
- `USE_FAKE_DATA` — if `True`, skips the real post-processing pipeline after capture (demo mode).
- `PLC_WAIT_FOR_WALL` — whether to block waiting for the "production line in position" PLC signal.
- `GET_MODEL_FROM_PLC` — whether to read wall index/model via the snap7 connection, vs. using a hardcoded
  fallback (`wall_index=1`, `wall_model="J4_2025-2-19_LINE"`) in [backend/server/fusion_server.py](backend/server/fusion_server.py).
- `CAM_EXT_PKL` / `TRAJ_EXT_PKL` — which calibration set under `Data/model_*/` is currently active (see below).
- `ROOT_FOLDER` — root for captured data and the SQLite database (`ROOT_FOLDER/.db/inspection.db`,
  DXF uploads under `ROOT_FOLDER/.db/dxf/`).

## Architecture / request flow

**Entry point**: `main.py` activates CloudComPy, then starts `FusionServerHandler.start_server()`
(aiohttp app, port 1337) on the asyncio event loop, alongside a console-input coroutine.

**`FusionServerHandler`** ([backend/server/fusion_server.py](backend/server/fusion_server.py)) is the single
stateful object tying everything together — one `HardwareManager` and (per active run) one `ProjectManager`.
Flow:

1. Frontend opens `/ws`. Server sends `"start"`, then reacts to JSON messages containing a `step` field.
2. When the client reports `step == 3`, the server kicks off `run_capture_process()` as a background task:
   - optionally waits for the PLC "wall in position" signal (`hardware_manager.plc_wait_for_prod_line`)
   - optionally reads wall index/model from the PLC, else uses the hardcoded defaults
   - creates a new `ProjectManager(wall_index, wall_model)` for this run
   - for each of 8 positions: `hardware_manager.move_to_and_capture(step)` (moves gantry via Modbus, captures
     both RVC cameras), stores the result on `ProjectManager`, and notifies the frontend over the websocket
   - writes a `WallResult` DB row, then (unless `USE_FAKE_DATA`) launches `post_process_coroutine()` which runs
     the measurement pipeline and tells the frontend to refresh
3. HTTP routes (all CORS-enabled) serve captured images, the combined preview image, DXF previews, exported
   Excel/results, and drive printing / starting a new project. See the route table at the bottom of
   `fusion_server.py::start_server` for the full list.

**`HardwareManager`** ([backend/hardware_manager.py](backend/hardware_manager.py)) is a thin simulation/real
switch (`config.RUN_SIMULATION`) over the PLC (`plc_backend/`) and dual RVC-X cameras (`rvc_cameras/`). Real-mode
imports of `PyRVC`/`snap7`-backed modules are conditional on `RUN_SIMULATION` so simulation mode works without
the hardware SDKs installed.

**`ProjectManager`** ([backend/project_manager.py](backend/project_manager.py)) represents one inspection run:

- Generates a daily-sequential inspection ID: `YYYYMMDD` + zero-padded count of today's existing DB rows.
- Resolves the DXF reference file by convention: the first `_`-separated token of `wall_model` + `.dxf`,
  looked up under `DXF_DIR`.
- `combine_pcds()` → `algorithms/calib_concant.py::combine_frames_extrinsic`: merges the 8×(left+right) captured
  point clouds using the active `CAM_EXT_PKL`/`TRAJ_EXT_PKL` transforms, plus fixed alignment matrices and a crop
  bounding box tuned for the physical rig — **not general-purpose, tied to this specific gantry setup**.
- `run_algorithms()` → `algorithms/measure_compare/measurement.py::all_measurement(pcd_path, dxf_path)`: the
  core CAD-vs-as-built comparison. Uses `cloudComPy` for CAD-aligned preprocessing/rotation/slicing, computes
  wall height/width/diagonal errors and window/door opening errors against fixed tolerances, and writes
  `results.json`, `results.xlsx` (filled from the `Data/results.xlsx` template), comparison PNGs, and
  `results.pdf` (via Excel COM automation).

**`algorithms/`** — image/point-cloud processing, independent of the server/hardware layers:
- `CCTDecoder/` — detects coded circular targets (CCT markers) in images, used for calibration correspondence.
- `my_pcd.py` — `MyPCD` wraps one captured folder (`PointCloud.ply` + `Image.png`), gives pixel→3D bilinear
  lookups and ICP refinement.
- `calib_concant.py` — combines captured frames into one point cloud via extrinsics (used at runtime).
- `pcd_convert_png.py` / `dxf_convert_png.py` — render point clouds / DXF files to 2D PNG previews.
- `measure_compare/` — the CAD-vs-point-cloud measurement pipeline (`dxf_analyze`, `preprocess`,
  `extract_slices`, `main_fit_bottom_line` for door/window hole detection, `main_cad_align`,
  `main_rotateToGetMinBB`, `main_rotate_to_xy_plane`), orchestrated by `measurement.py`.

**`calib/`** — offline calibration tooling (not part of the runtime server loop). `calib.py` computes the
inter-camera / inter-position transform matrices (using CCT marker correspondences) that get pickled into
`Data/model_*/left_right_ext.pkl` and `cam_traj_ext.pkl` for a given rig/model configuration.

**`Data/model_*/`** — each folder is a dated calibration set (`left_right_ext.pkl`, `cam_traj_ext.pkl`) that
must match the physical camera/gantry calibration currently in use; `config.py`'s `CAM_EXT_PKL`/`TRAJ_EXT_PKL`
select which one is active.

**`backend/inspect_db.py`** — SQLite via peewee. Single `WallResult` table (id, frame_folder, dxf_filename,
wall_index, wall_model, created_date) tracks each inspection run.

**`fake/`** — demo assets used when `USE_FAKE_DATA=True`.
