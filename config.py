"""
This file contains basic configuration options for the application to work normally.
Please change according to the systems' file sturcture.
"""


############################ Simulation configuration ############################
# Control whether PLC not to wait for wall in position signal.
PLC_WAIT_FOR_WALL = True
# Control whether to connect to PLC and camera.
RUN_SIMULATION = False
SIMULATION_DATA_DIR = r"D:\Inspection_data\20250309001"
USE_FAKE_DATA = False
GET_MODEL_FROM_PLC = False


############################ Capture geometry ############################
# Number of gantry stops the wall is captured from, i.e. how many positions the
# capture loop visits. MUST match the PLC's iCameraNum (D906) -- the PLC divides
# its travel range into that many stops and ignores any commanded index above it.
# It must also match the number of keys in the active TRAJ_EXT_PKL below.
# (Was 8 on the previous project; this rig is 7.)
NUM_CAPTURE_POSITIONS = 7


############################ Extrinsics configuration ############################
# extrinsic between left and right camera AND extrinsic between each of two right capture position. 
CAM_EXT_PKL = r"Data\model_0308\left_right_ext.pkl"
TRAJ_EXT_PKL = r"Data\model_0308\cam_traj_ext.pkl"


############################ Frontend serving ############################
# Built frontend (Vite `npm run build` output) served by aiohttp at "/", so the
# Electron shell can load http://127.0.0.1:1337/ and everything is same-origin --
# no dev-server proxy and no CORS in production.
# Point this at the `dist` folder of the ensightful-control-electron checkout.
# Set to None to disable static serving (API-only, e.g. when using `npm run dev`).
FRONTEND_DIST_DIR = r"C:\Users\yuany\ensightful-control-electron\dist"


############################ Capture Saving Options ############################
# root folder for saving captured data and database files.
# ROOT_FOLDER = r"U:\Inspection_Data"
ROOT_FOLDER = r"D:\Inspection_Data"

############################ PLC Options ############################
# Gantry motion PLC (Modbus TCP). Found via AutoShop's 通讯设置 -> 以太网 -> 搜索,
# which lists every PLC on the network with its IP/MAC; ping it to confirm.
# Note AutoShop may be connected to the PLC over USB for programming — that says
# nothing about Ethernet, which is what Modbus TCP needs. Modbus TCP also has to
# be enabled on the PLC itself: a successful ping only proves the PLC is on the
# network, not that port 502 answers.
# (Was 192.168.111.3, inherited from the previous project's rig.)
PLC_HOST = '192.168.124.88'
PLC_PORT = 502
