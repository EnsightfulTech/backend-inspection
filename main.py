"""
Main file for the inspection project.
"""
from envActivation import setup_environment
setup_environment()

from loguru import logger

# CloudComPy is only needed by the measurement pipeline (CAD comparison, slicing,
# report rendering). Capture, the PLC, the websocket and the whole UI work fine
# without it, so a missing/broken install must NOT stop the server from starting
# -- same principle as hardware bring-up: come up, report what is degraded, and
# fail clearly only when the missing feature is actually used.
#
# Practical effect: a dev machine without the CloudComPy binary bundle can still
# run the server, the UI, and simulation-mode captures.
CLOUDCOMPY_AVAILABLE = False
CLOUDCOMPY_ERROR = None
try:
    import cloudComPy as cc
    cc.initCC()
    CLOUDCOMPY_AVAILABLE = True
    logger.success("CloudComPy initialised")
except Exception as e:
    CLOUDCOMPY_ERROR = str(e)
    logger.error(
        f"CloudComPy unavailable ({e}). The server will start and capture will "
        f"work, but post-processing (measurement, CAD comparison, Excel/PDF "
        f"export) will fail. Check envActivation.py's CloudComPy path and that "
        f"this interpreter is Python 3.10 x64.")

from backend.async_io import AsyncIO
from backend.server.fusion_server import FusionServerHandler
import asyncio
import traceback

async def main():
    fusion_server = FusionServerHandler()
    task1 = asyncio.create_task(fusion_server.start_server())
    test = AsyncIO()
    task2 = asyncio.create_task(test.async_input('input: '))
    await asyncio.gather(task1, task2)

try:
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())
    loop.run_forever()
except BaseException as e:
    e = traceback.format_exc(); logger.error(e)
    logger.info("Server stopped")
    loop.stop()
    pass
