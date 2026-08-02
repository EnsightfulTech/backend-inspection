# Runtime environment (Windows, Python 3.10)

Verified end-to-end on 2026-08-02. Follow this exactly and the backend will run;
deviating on the Python version or the conda/pip split will fail in ways that are
hard to diagnose.

## Why it has to be one environment

`main.py` imports **`cloudComPy`** (measurement pipeline) and, via
`hardware_manager`, **`PyRVC`** (cameras) — in the same interpreter. They cannot
live in separate environments.

Both are compiled extensions locked to **Python 3.10 x64**:
- `PyRVC.cp310-win_amd64.pyd`
- `_cloudComPy.cp310-win_amd64.pyd`

`open3d` additionally requires `numpy < 2`. So: Python 3.10, one env, numpy pinned.

## Verified versions

```
python  3.10.13   numpy  1.26.4   cv2  4.8.1 (conda)   open3d  0.19.0
qt 5.15.8   pcl 1.11.1   boost 1.74.0   cgal 5.4   gdal 3.8.0
CloudComPy310 binary: 2024-09-27 (CloudCompare 2.13.2)
PyRVC 1.15.0 (built from the SDK source; see calib/CALIBRATION.md)
```

## 1. CloudComPy binary

Not pip-installable and **not** part of the normal CloudCompare install — it is a
separate distribution. Download the **Python 3.10** build (releases from 2025 and
earlier; current ones target 3.12) from
<https://www.simulation.openfields.fr/index.php/cloudcompy-downloads>.

Unpack so `envCloudComPy.bat` sits directly in `C:\workspace\CloudComPy310`:

```
C:\workspace\CloudComPy310\
    envCloudComPy.bat
    CloudCompare\        <- _cloudComPy.cp310-win_amd64.pyd lives here
    doc\PythonAPI_test\
    ccViewer\
```

That exact path is hardcoded in [envActivation.py](envActivation.py); unpack
elsewhere and you must edit `batch_file_path` there.

Note the bundle ships **no Qt, boost, CGAL or gdal** — those come from conda,
which is why the env below is not optional.

## 2. Conda layer

From CloudComPy's own `doc/UseWindowsCondaBinary.md`, **plus `mpir`** (see below):

```bat
conda create -y --name CloudComPy310 python=3.10
conda activate CloudComPy310
conda config --add channels conda-forge
conda config --set channel_priority flexible
conda install -y boost cgal cmake draco "ffmpeg=6.1" gdal jupyterlab laszip matplotlib "mysql=8" notebook numpy opencv openmp "openssl=3.1" pcl pdal psutil pybind11 quaternion "qhull=2020.2" "qt=5.15.8" scipy sphinx_rtd_theme spyder tbb tbb-devel "xerces-c=3.2"
conda install -y mpir
```

### ⚠️ `mpir` is missing from CloudComPy's documented list

Without it the import fails with `DLL load failed while importing _cloudComPy`.
MPIR is a multi-precision arithmetic library CGAL links against; conda's `cgal`
does not declare it as a runtime dependency (CGAL is largely header-only), so it
is not pulled in and not mentioned upstream. It was the **only** unresolved
dependency in the whole bundle.

**Checkpoint — do not continue until this passes:**

```bat
cd /d C:\workspace\CloudComPy310
envCloudComPy.bat
```
Expect `Environment OK!`.

## 3. Pip layer (after conda, never before)

Conda owns `numpy`, `opencv`, `matplotlib`, `scipy` — do **not** pip-install
those, or you will end up with two managers fighting over the same libraries.
So this is `requirements.txt` minus those four, plus `pywin32`:

```bat
pip install aiohttp aiohttp_cors loguru aiofiles peewee icecream aioconsole ^
            pyModbusTCP open3d Pillow ezdxf openpyxl pandas windows-toasts ^
            python-snap7 pywin32
pip install <path>\pyrvc-1.15.0-cp310-cp310-win_amd64.whl
```

`pywin32` is needed for the Excel→PDF export in `measurement.py` and is absent
from `requirements.txt`.

## 4. Verify

```bat
python -c "import cloudComPy, PyRVC; print('both OK')"
python envActivation.py          :: expect "Environment OK!"
python -c "import numpy; print(numpy.__version__)"   :: must be < 2
```

`import cloudComPy` prints two harmless Qt messages
(`QFileSystemWatcher: ... no QCoreApplication instance`, `JsonRPCPlugin::...`) —
CloudCompare initialising its plugin system headlessly. Not errors.

Then point the Electron shell at this interpreter — `BACKEND_PYTHON`, or
`"python"` in `backend.config.json` (see [INTEGRATION.md](INTEGRATION.md)):

```
C:\Users\<user>\miniconda3\envs\CloudComPy310\python.exe
```

## Troubleshooting

### `checkenv.py` hides the real error

CloudComPy's `checkenv.py` catches the import with a bare `except:` and prints
`"The environment seems to be incorrect!"` regardless of cause. That message says
nothing about what actually failed. Get the real exception:

```python
import os, sys, traceback
ROOT = r"C:\workspace\CloudComPy310"
sys.path.insert(0, os.path.join(ROOT, "CloudCompare"))
sys.path.insert(0, os.path.join(ROOT, "doc", "PythonAPI_test"))
try:
    import cloudComPy
    print("OK")
except Exception:
    traceback.print_exc()
```

- `ModuleNotFoundError: No module named 'cloudComPy'` → extraction layout wrong
- `ImportError: DLL load failed while importing _cloudComPy` → a dependent DLL is
  missing; find it as below

### Finding a missing DLL

`dumpbin` ships with Visual Studio. This lists every dependency of the bundle
that cannot be resolved:

```powershell
$dumpbin = "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Tools\MSVC\14.44.35207\bin\Hostx64\x64\dumpbin.exe"
$cc = "C:\workspace\CloudComPy310\CloudCompare"
$libbin = "C:\Users\<user>\miniconda3\envs\CloudComPy310\Library\bin"
$search = @($cc, "$cc\plugins", $libbin, "C:\Windows\System32")
$targets = @("$cc\_cloudComPy.cp310-win_amd64.pyd") + (Get-ChildItem $cc -Filter *.dll).FullName
$all = @{}
foreach ($t in $targets) {
  & $dumpbin /dependents $t 2>$null | Select-String "^\s+\S+\.dll$" |
    ForEach-Object { $all[$_.Line.Trim()] = $true }
}
$all.Keys | Where-Object { $d = $_
  $d -notlike "api-ms-*" -and $d -notlike "ext-ms-*" -and
  -not ($search | Where-Object { Test-Path (Join-Path $_ $d) })
} | Sort-Object
```

An empty result means every dependency resolves. This is how `mpir.dll` was
found — install the conda package providing whatever it names.

### China deployment

Several of these downloads fail or stall from a mainland-China network. Mirrors:

```bat
pip config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple
conda config --add channels https://mirrors.tuna.tsinghua.edu.cn/anaconda/cloud/conda-forge
```

Binaries that must be fetched elsewhere and copied across, since no mirror helps:
the CloudComPy `.7z`, the PyRVC wheel, and Electron's binary (frontend repo).
Keep all three with the deployment artifacts rather than re-solving each time.
