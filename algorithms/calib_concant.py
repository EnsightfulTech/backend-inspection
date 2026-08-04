import numpy as np
from icecream import ic
from typing import List
from pathlib import Path
from loguru import logger
import pickle
import open3d as o3d

from .my_pcd import MyPCD

TRANS1 = np.array([
    [0.992081165314, -0.000635988603, 0.125596761703, -0.332078039646],
    [-0.000635988603, 0.999948918819, 0.010087110102, -0.026670455933],
    [-0.125596761703, -0.010087110102, 0.992030084133, -0.037964105606],
    [0.000000000000, 0.000000000000, 0.000000000000, 1.000000000000]
])

TRANS2 = np.array([
    [0.999994099140, 0.000002645179, -0.003429675940, 0.008911013603],
    [0.000002645179, 0.999998807907, 0.001542518614, -0.004007816315],
    [0.003429675940, -0.001542518614, 0.999992907047, -0.001772880554],
    [0.000000000000, 0.000000000000, 0.000000000000, 1.000000000000]
])

TRANS3 = np.array([
    [0.992054939270, -0.000522068643, 0.125804364681, -0.331498384476],
    [-0.000522068643, 0.999965667725, 0.008266577497, -0.021782636642],
    [-0.125804364681, -0.008266577497, 0.992020606995, -0.051970720291],
    [0.000000000000, 0.000000000000, 0.000000000000, 1.000000000000]
])


def crop_static_bound(pcd, min_bound, max_bound):
    cropping_box = o3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound)
    cropped_pcd = pcd.crop(cropping_box)

    return cropped_pcd

def resample_pcd(pcd, number_ratio=1/2):
    logger.info(f"resampling pcd with ratio {number_ratio}")
    pcd_down = pcd.random_down_sample(number_ratio)
    return pcd_down

def combine_frames_extrinsic(combine_folder, cam_ext_pkl, traj_ext_pkl, crop_bounds=None):
    cam_ext = pickle.load(open(cam_ext_pkl, "rb"))
    traj_ext = pickle.load(open(traj_ext_pkl, "rb"))
    logger.info(f"loaded cam_ext and traj_ext from {cam_ext_pkl} and {traj_ext_pkl}")

    # glob all the folders
    frames_folder = [f for f in sorted(combine_folder.iterdir()) if f.is_dir()]
    ic(frames_folder)

    # Concatenate the point clouds
    pcd_combined = o3d.geometry.PointCloud()

    for frame_path in frames_folder:
        frame_path: Path
        idx = frame_path.name
        logger.info(f"Combining {idx} frame ...")
        rt_cam_traj= traj_ext[idx]
        rt_lr = cam_ext

        left_frame = MyPCD(frame_path / "left")
        right_frame = MyPCD(frame_path / "right")

        left_pcd = left_frame.pcd
        right_pcd = right_frame.pcd

        pcd_combined += left_pcd.transform(rt_cam_traj)
        pcd_combined += right_pcd.transform( rt_cam_traj @ rt_lr )

    # RVC captures contain NaN points for invalid-depth pixels (sensor
    # holes/no-return). The old hardcoded crop_static_bound() below used to
    # discard these as an accidental side effect (a bounding-box crop test is
    # false for NaN coordinates), which masked this: get_min_bound()/
    # get_max_bound() propagate NaN if any point is NaN, which breaks
    # convert_pcd_to_2d_image()'s int(y_range * ...) with "cannot convert
    # float NaN to integer" once nothing upstream filters them out anymore.
    n_before = len(pcd_combined.points)
    pcd_combined.remove_non_finite_points()
    n_after = len(pcd_combined.points)
    if n_after != n_before:
        logger.info(f"dropped {n_before - n_after} non-finite points "
                    f"({n_before} -> {n_after})")

    # transform pcd_combined by TRANS2@TRANS1
    pcd_combined.transform(TRANS2@TRANS1)
    # rotate pcd_combined around z-axis by 180 degrees
    rt_180 = np.array([
        [-1, 0, 0, 0],
        [0, -1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    pcd_combined.transform(rt_180)

    # crop static bound: opt-in only. The previous hardcoded bound here
    # (z in [2.55, 2.73], an 18cm slice) was tuned for a different wall/rig
    # setup -- on this rig it clips away nearly the entire cloud, leaving
    # just the sliver that happens to intersect that range ("a stripe").
    # The measurement pipeline doesn't depend on this crop being tight: it
    # does its own adaptive re-orientation and Z-range detection from the
    # real data (preprocess.py: extract_main_cc, cal_z_range). Pass
    # crop_bounds=(min_bound, max_bound) once this rig's real working Z-range
    # is known, if a tighter preview crop is wanted again.
    if crop_bounds is not None:
        pcd_combined = crop_static_bound(pcd_combined, *crop_bounds)


    pcd_combined = resample_pcd(pcd_combined)
    logger.info(f"pcd_combined has {len(pcd_combined.points)} points")

    return pcd_combined



if __name__ == '__main__':

    combine_folder = Path(r"C:\Users\14904\Data")
    cam_ext_pkl = Path(r"C:\workspace\Data_test\0925_cam_ext\left_right_ext.pkl")
    traj_ext_pkl = Path(r"C:\workspace\Data_test\0925_traj_ext\cam_traj_ext.pkl")

    pcd_combined = combine_frames_extrinsic(combine_folder, cam_ext_pkl, traj_ext_pkl)

    o3d.io.write_point_cloud("combined_mic_0927_03pm.ply", pcd_combined)

    # crop(pcd_combined)
