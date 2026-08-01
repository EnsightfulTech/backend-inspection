# Frontend ↔ backend integration

How the Electron UI (`ensightful-control-electron`) and this Python backend run
together as one standalone-feeling application on the rig PC.

## Why a split at all

The client's objection — "the rig PC is standalone, why is there a client and a
server" — is about **user experience**, not architecture. The split is forced:
`cloudComPy` and `PyRVC` are precompiled binaries ABI-locked to Python 3.10 x64,
and the measurement pipeline is a few thousand lines of numpy/open3d/CloudCompare
work. None of that can move into the renderer.

So the design goal is not to remove the split — it is to make it **invisible**:
one icon, one window, one thing to close, and no state where the operator sees
"cannot reach server" on a machine talking to itself.

A side benefit worth keeping: process isolation. A CloudComPy segfault
mid-measurement does not take the UI down, and a renderer crash does not abort a
capture in progress.

## Shape

```
Electron main process  (electron/main.ts)
  ├── splash window            status + fallback buttons
  ├── spawn  <python.exe> main.py     (cwd = this repo)
  ├── poll   GET /health  until it answers
  ├── show   win.loadURL('http://127.0.0.1:1337/')
  └── quit   kill the backend process tree

aiohttp  (main.py -> FusionServerHandler)
  ├── /ws, /imageList, /leftImage, /health, /reconnectHardware, ...
  └── /            -> static FRONTEND_DIST_DIR   (the built Vue app)
```

The renderer is served **by the backend**, so the UI and the API are
same-origin. That removes three problems at once: the dev-server proxy is not
needed in production, CORS does not apply, and `file://`-relative API URLs (which
silently broke every HTTP call in a packaged build) are gone.

## Two runtimes

|  | Development | Production (rig) |
|---|---|---|
| UI served by | Vite dev server (HMR) | aiohttp, from `FRONTEND_DIST_DIR` |
| Electron loads | `VITE_DEV_SERVER_URL` | `http://127.0.0.1:1337/` |
| axios `baseURL` | `/api` (proxied) | `""` (same-origin) |
| WebSocket | `ws://<host>/api/ws` (proxied, `ws: true`) | `ws://<host>/ws` |

Both are derived from `location` in `src/config/backend.ts` — **no host is
hardcoded anywhere**. Previously `192.168.10.14:1337` appeared in five separate
places, which broke as soon as both halves ran on the same PC.

## Configuration

### Backend (`config.py`)
- `FRONTEND_DIST_DIR` — path to the frontend's `dist/`. Set to `None` to serve
  API-only (what `npm run dev` wants). A missing or unbuilt `dist` is not an
  error; it logs a warning and continues API-only.

### Electron (environment variables, all optional)
| Variable | Default | Purpose |
|---|---|---|
| `BACKEND_URL` | `http://127.0.0.1:1337` | where the backend listens |
| `BACKEND_PYTHON` | `…\envs\wallInspect\python.exe` | interpreter that can import `cloudComPy`/`PyRVC` |
| `BACKEND_CWD` | `C:\workspace\backend_inspection` | this repo's checkout |
| `BACKEND_SPAWN` | `true` | `false` to attach to a backend you started yourself |
| `BACKEND_BOOT_TIMEOUT_MS` | `90000` | how long to wait before offering fallback |

Nothing needs a rebuild to reconfigure a machine.

## Boot sequence and failure handling

1. Splash appears immediately.
2. `GET /health` once — if a backend is **already** running (dev, or a manual
   launch), attach to it rather than spawning a second one that would fail to
   bind port 1337.
3. Otherwise spawn the backend; its stdout/stderr are captured for the splash.
4. Poll `/health` until it answers or `BACKEND_BOOT_TIMEOUT_MS` elapses.
5. On success, load the UI and close the splash.
6. On failure, the splash shows the last backend output plus two buttons:
   - **重试连接** — re-spawn and re-poll
   - **仍然打开界面** — open the UI anyway (the operator may know the backend is
     just slow, or may want to read the UI's own error messages)

On quit, the backend process **tree** is killed (`taskkill /T /F` on Windows) —
otherwise a hidden python process keeps holding the cameras and port 1337. Only
a backend this app spawned is killed; one started manually is left alone.

## Health and hardware recovery

`GET /health` never returns non-200 for degraded hardware — the whole point is
that the UI can load and explain the problem:

```jsonc
{ "success": true, "data": {
    "backend": "ok", "capture_running": false, "ws_connected": true,
    "hardware": {
      "ready": false,                       // can a capture actually run?
      "simulation": false,
      "expected_stops": 7,
      "plc":     { "ok": false, "host": "192.168.124.88:502",
                   "error": "...", "camera_num": null },
      "cameras": { "ok": true, "error": null }
    } } }
```

`GET /reconnectHardware` re-runs hardware bring-up and returns the same status —
this is what a "retry hardware" control in the UI should call. It refuses while a
capture is running rather than re-initialising the camera SDK underneath it.

Hardware initialisation is **non-fatal by design**. Each subsystem is brought up
independently and failures are recorded, not raised, so the server always starts
and the UI can always load. Operations that need missing hardware raise
`HardwareNotReady` with a clear message instead of `AttributeError` on `None`.

## Deploying to the rig

```bash
# 1. frontend: build once, the backend serves the output
cd ensightful-control-electron
npm install && npm run build:vite        # produces dist/

# 2. backend: point config.py at that dist/
#    FRONTEND_DIST_DIR = r"...\ensightful-control-electron\dist"

# 3. package the shell (or run `npm run dev` during development)
npm run electron:build:win
```

Do **not** try to bundle Python into the installer — PyInstaller and
`cloudComPy`'s Qt/PCL DLL bundle will fight you. Keep the conda env installed on
the rig and point `BACKEND_PYTHON` at it, exactly as `run_backend.bat` does today.

## Troubleshooting

| Symptom | Cause |
|---|---|
| Splash stuck, no backend output | `BACKEND_PYTHON` / `BACKEND_CWD` wrong — shown on the splash |
| Splash times out, backend log shows a traceback | backend crashed on import; run `python main.py` by hand to see it |
| UI loads but blank | `FRONTEND_DIST_DIR` unset/unbuilt — backend logs "serving API only" |
| UI loads, data never appears | backend up but hardware not ready — check `/health` |
| Capture never starts | PLC preconditions: AUTO mode, homed, valid travel range, `iCameraNum > 1` |
