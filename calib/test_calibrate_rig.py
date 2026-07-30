"""
Verification for rig_geometry + calibrate_rig, pure numpy/scipy (no camera stack).

Run:  python test_calibrate_rig.py
Passes silently with a summary; raises AssertionError on failure.

Covers:
  1. rig_geometry unit checks (umeyama recovery, vec<->mat + invert round-trips).
  2. Synthetic end-to-end: build a rig with a known left-right extrinsic and a known
     smooth pose(x) rail-sag, place a fixed uniquely-ID'd marker field, "capture" it
     from each stop with both cameras (adding depth noise), run bundle_adjust, and
     assert the recovered T_lr and per-stop poses match ground truth within noise.
     Also checks that omitting stitch-across-passes still recovers, and that pose(x)
     evaluation at operational positions is accurate.
"""

import os
import sys

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from calib import rig_geometry as g
from calib.calibrate_rig import Obs, bundle_adjust, fit_and_eval_posex


def _pose(rotvec, t):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_rotvec(rotvec).as_matrix()
    T[:3, 3] = t
    return T


def test_geometry():
    rng = np.random.default_rng(1)
    T = _pose([0.02, -0.03, 0.05], [1580.0, -4.0, 9.0])
    src = rng.normal(0, 700, (50, 3))
    dst = g.apply(T, src) + rng.normal(0, 0.15, (50, 3))
    That = g.umeyama(src, dst)
    dR = np.degrees(np.linalg.norm(Rotation.from_matrix(That[:3, :3] @ T[:3, :3].T).as_rotvec()))
    dt = np.linalg.norm(That[:3, 3] - T[:3, 3])
    assert dR < 0.05, f"umeyama rotation err {dR} deg"
    assert dt < 1.0, f"umeyama translation err {dt} mm"

    v = g.mat_to_vec(T)
    assert np.allclose(g.vec_to_mat(v), T, atol=1e-9)
    assert np.allclose(g.invert(T) @ T, np.eye(4), atol=1e-9)
    print(f"[geometry] OK  umeyama dR={dR:.4f}deg dt={dt:.3f}mm")


def _sag_truth(x):
    """Ground-truth stop pose (left-cam -> reference) as a smooth function of rail x.
    Reference is x=0. Pitch/yaw grow ~ mid-span bow; translation mostly along x."""
    # normalized mid-span bow in [0,1] over x in [0,6000]
    bow = np.sin(np.pi * x / 6000.0)
    rotvec = np.array([0.004 * bow,        # roll grows toward mid-span
                       -0.006 * bow,       # pitch (nod) toward mid-span
                       0.002 * bow])       # small yaw
    t = np.array([float(x),                # travels along x
                  3.0 * bow,               # slight lateral wander
                  -8.0 * bow])             # droop (z) toward mid-span
    return _pose(rotvec, t)


def test_end_to_end():
    rng = np.random.default_rng(7)

    # --- ground truth rig ---
    T_lr_true = _pose([0.01, 0.015, -0.008], [1583.4, 2.0, -3.0])  # right -> left
    stop_x = {f"stop_{i:02d}": x for i, x in enumerate(np.linspace(0, 6000, 7))}
    stop_names = sorted(stop_x)
    ref_stop = stop_names[0]

    # reference-frame poses; T_s maps stop-s left-cam -> reference, so a point given
    # in reference frame is seen by stop s's left cam at  inv(T_s) @ X_ref.
    T_stop_true = {s: _sag_truth(stop_x[s]) for s in stop_names}
    # rebase so reference stop is identity (matches solver gauge)
    base = g.invert(T_stop_true[ref_stop])
    T_stop_true = {s: base @ T_stop_true[s] for s in stop_names}

    depth_noise = 0.4  # mm, per-point (depth-sensor-like)

    # --- fixed marker field per pass; boards re-posed between passes ---
    obs = []
    n_passes = 3
    for p in range(n_passes):
        pass_id = f"pass_{p:02d}"
        # a strip of uniquely-ID'd markers along x, spread in y, at a couple heights
        mid = 0
        for mx in np.linspace(200, 5800, 24):
            for my in (-700, 0, 700):
                for mz in (0.0, 300.0):       # two heights -> breaks coplanarity
                    # per-pass jitter of the whole field (re-posed boards)
                    G = np.array([mx + rng.normal(0, 40),
                                  my + rng.normal(0, 40),
                                  mz + rng.normal(0, 20)])
                    mid += 1
                    marker_id = p * 100000 + mid   # unique within pass
                    # which stops see this point? emulate FOV: |x_stop - mx| <= 1072
                    for s in stop_names:
                        if abs(stop_x[s] - mx) > 1072.0:
                            continue
                        # point in stop-s left-cam frame:
                        X_left = g.apply(g.invert(T_stop_true[s]), G[None, :])[0]
                        X_left_n = X_left + rng.normal(0, depth_noise, 3)
                        obs.append(Obs(pass_id, s, "L", marker_id, X_left_n))
                        # right cam sees it too if within the L-R overlap wedge (|y|<566 near floor)
                        if abs(my) <= 566:
                            X_right = g.apply(g.invert(T_lr_true), X_left[None, :])[0]
                            X_right_n = X_right + rng.normal(0, depth_noise, 3)
                            obs.append(Obs(pass_id, s, "R", marker_id, X_right_n))

    print(f"[e2e] synthesized {len(obs)} observations over {n_passes} passes")

    result = bundle_adjust(obs, stop_names, ref_stop, verbose=False)

    # --- check T_lr ---
    T_lr = result["T_lr"]
    dR_lr = np.degrees(np.linalg.norm(
        Rotation.from_matrix(T_lr[:3, :3] @ T_lr_true[:3, :3].T).as_rotvec()))
    dt_lr = np.linalg.norm(T_lr[:3, 3] - T_lr_true[:3, 3])
    print(f"[e2e] T_lr err: dR={dR_lr:.4f}deg dt={dt_lr:.3f}mm")
    assert dR_lr < 0.05, f"T_lr rotation err too high: {dR_lr} deg"
    assert dt_lr < 2.0, f"T_lr translation err too high: {dt_lr} mm"

    # --- check per-stop poses ---
    max_dt, max_dR = 0.0, 0.0
    for s in stop_names:
        Te, Tt = result["T_stops"][s], T_stop_true[s]
        dR = np.degrees(np.linalg.norm(
            Rotation.from_matrix(Te[:3, :3] @ Tt[:3, :3].T).as_rotvec()))
        dt = np.linalg.norm(Te[:3, 3] - Tt[:3, 3])
        max_dt, max_dR = max(max_dt, dt), max(max_dR, dR)
    print(f"[e2e] per-stop max err: dR={max_dR:.4f}deg dt={max_dt:.3f}mm")
    assert max_dR < 0.05, f"stop rotation err too high: {max_dR} deg"
    assert max_dt < 3.0, f"stop translation err too high: {max_dt} mm"
    print(f"[e2e] BA overall residual RMS = {result['stats']['overall_rms_mm']:.3f} mm "
          f"(depth noise was {depth_noise} mm)")

    # --- pose(x) model evaluated at 'operational' positions (same 7 here) ---
    op = {f"{i+1:02d}": stop_x[s] for i, s in enumerate(stop_names)}
    _, traj_ext, crosscheck = fit_and_eval_posex(
        result, stop_x, op, model_kind="poly", poly_deg=4,
        crosscheck_tol_mm=3.0, crosscheck_tol_deg=0.05)
    assert set(traj_ext.keys()) == set(op.keys()), "operational keys mismatch"
    assert all(c["ok"] for c in crosscheck), f"pose(x) cross-check failed: {crosscheck}"
    print(f"[e2e] pose(x) cross-check passed at {len(crosscheck)} operational stops")


if __name__ == "__main__":
    test_geometry()
    test_end_to_end()
    print("\nALL TESTS PASSED")
