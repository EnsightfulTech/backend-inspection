"""
calibrate_rig.py — joint calibration of the dual-camera gantry inspection rig.

Solves, in ONE global bundle adjustment pooled over multiple capture passes:
  * the left-right camera extrinsic  T_lr  (right-cam -> left-cam), shared by all stops
  * every stop's trajectory pose      T_s   (stop-s left-cam -> reference frame)
with the reference stop fixed as identity (the common frame / gauge).

It then fits each pose DOF as a smooth pose(x) curve vs rail position, plots the
sag profile, evaluates the model at the operational stops, and writes extrinsics
in the exact pickle format the inspection runtime
(`algorithms/calib_concant.py::combine_frames_extrinsic`) consumes:
    left_right_ext.pkl : single 4x4  (right -> left)
    cam_traj_ext.pkl   : {stop_name: 4x4}  (that stop's left-cam -> reference)

Why this replaces the old tooling
---------------------------------
`optimize.py::generate_pkl_by_using_oneRT` propagated a single constant step
transform (assumes every inter-stop move is identical); the rail sags in mid-span
so that accumulates error. Here every stop is a free pose tied together by
loop-closure markers and solved jointly, and the sag is modeled explicitly.

The bundle-adjustment core (`bundle_adjust`) is pure numpy/scipy and imports no
camera/open3d code, so it can be unit-tested on synthetic data. The camera-stack
imports (MyPCD, CCT_extract) are done lazily inside `load_observations`.

Data layout expected (pass-major):
    calib_root/
      pass_00/ stop_00/{left,right}/{Image.png,PointCloud.ply}
               stop_01/...
      pass_01/ stop_00/...        (boards re-posed between passes)
      ...
"""

import argparse
import json
import os
import pickle
import sys
from collections import defaultdict, namedtuple
from pathlib import Path

# make the repo root importable whether run as `python calib/calibrate_rig.py`
# or `python -m calib.calibrate_rig` (so `algorithms.*` / `calib.*` resolve).
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from calib import rig_geometry as g

# one detected marker lifted to 3D in its own camera frame
Obs = namedtuple("Obs", ["pass_id", "stop", "cam", "marker_id", "xyz"])

# Point clouds loaded via algorithms.my_pcd.MyPCD are in METERS (RVC saves
# PointMapUnitEnum.Meter; verified against a real capture: z/depth median
# ~2.89, matching the rig's known ~2.85m camera standoff). Every function below
# (bundle_adjust, residual_report, fit_and_eval_posex) is unit-agnostic and
# labels its outputs "_mm" / compares against tolerances named "*_tol_mm" --
# i.e. it assumes its input Obs.xyz values are already millimeters, matching
# stop_x (from the PLC, genuinely mm) and test_calibrate_rig.py's synthetic
# data (constructed directly in mm). So the ONE place that must convert is
# load_observations(), right where real point-cloud xyz is lifted -- not
# scattered through the math/reporting functions, which stay unit-agnostic and
# are shared by the synthetic test. The inspection runtime
# (algorithms/calib_concant.py::combine_frames_extrinsic) then consumes
# left_right_ext.pkl/cam_traj_ext.pkl against point clouds that are natively
# METERS again, so write_outputs() converts back mm -> meters only for those
# two pickles, not for calib_report.json (which stays mm, for humans).
M_TO_MM = 1000.0

# The physical CCT boards' printed layout ("双目二维码61x52cm-16张.pdf"): 16
# boards of 9 codes each, 144 unique codes total. Verified against the design
# file -- no duplicate codes across boards. Used by _screen_detections() as a
# structural prior: real markers always appear in dense boards of 9, so a
# decode with no board-mates nearby, or one that contradicts its neighbors'
# board identity, is almost certainly wrong -- confirmed by visual inspection
# (see CALIBRATION.md) against real captures: every misdecode found either (a)
# had zero recognizable board-member codes nearby (pure false positive on
# background clutter, e.g. rebar/decking mistaken for a target), or (b) sat on
# a board where 2+ of the other 8 cells decoded correctly and its own decode
# didn't match the one remaining unclaimed code for that board.
BOARD_LAYOUT = [
    (479, 485, 487, 489, 491, 493, 495, 499, 501),
    (503, 505, 507, 509, 511, 585, 587, 589, 591),
    (595, 597, 599, 603, 605, 607, 613, 615, 619),
    (621, 623, 627, 629, 631, 635, 637, 639, 661),
    (663, 667, 669, 671, 679, 683, 685, 687, 691),
    (693, 695, 699, 701, 703, 715, 717, 719, 723),
    (725, 727, 731, 733, 735, 743, 747, 749, 751),
    (755, 757, 759, 763, 765, 767, 819, 821, 823),
    (827, 829, 831, 845, 847, 853, 855, 859, 861),
    (863, 871, 875, 877, 879, 885, 887, 891, 893),
    (895, 925, 927, 939, 941, 943, 949, 951, 955),
    (957, 959, 975, 981, 983, 987, 989, 991, 1003),
    (1005, 1007, 1013, 1015, 1019, 1021, 1023, 1365, 1367),
    (1371, 1375, 1387, 1391, 1399, 1403, 1407, 1455, 1463),
    (1467, 1471, 1495, 1499, 1503, 1519, 1527, 1531, 1535),
    (1755, 1759, 1775, 1783, 1791, 1911, 1919, 1983, 2015),
]
PRINTED_CODES = frozenset(c for board in BOARD_LAYOUT for c in board)
CODE_TO_BOARD = {c: i for i, board in enumerate(BOARD_LAYOUT) for c in board}
assert len(PRINTED_CODES) == sum(len(b) for b in BOARD_LAYOUT), "duplicate code across boards"


def log(msg):
    print(f"[calibrate_rig] {msg}", flush=True)


# ======================================================================== #
#  Decode screening: board-layout dictionary + isolation check
# ======================================================================== #
def _cluster_detections(detections, cluster_dist):
    """Greedy single-link clustering of (code, x, y) detections by pixel
    distance -- groups cells belonging to the same physical board together."""
    n = len(detections)
    parent = list(range(n))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    def union(i, j):
        ri, rj = find(i), find(j)
        if ri != rj:
            parent[ri] = rj

    pts = np.array([[x, y] for _, x, y in detections], dtype=float)
    for i in range(n):
        for j in range(i + 1, n):
            if np.linalg.norm(pts[i] - pts[j]) <= cluster_dist:
                union(i, j)

    clusters = defaultdict(list)
    for i in range(n):
        clusters[find(i)].append(i)
    return list(clusters.values())


def screen_detections(detections, cluster_dist=450.0, min_board_votes=2):
    """
    Validate/correct raw (code, x, y) CCT detections from one image against
    the known board layout.

    For each spatial cluster of detections (same physical board):
      - if >= min_board_votes decode to codes from the SAME board, that
        cluster is confidently identified as that board;
      - any other detection in the cluster that decodes to a code NOT on that
        board is either corrected (if it's the cluster's one unclaimed slot on
        that board -- "we see 1365, 1019, 1367, so the 4th detection must be
        1021") or dropped (if the correction would be ambiguous);
      - clusters that never reach min_board_votes are dropped entirely -- a
        real board always shows several markers together, so an unconfirmed
        lone detection (whether it's a misdecode or a false positive on
        background clutter) has no corroboration and isn't trustworthy.

    Returns a filtered/corrected list of (code, x, y).
    """
    if not detections:
        return []

    out = []
    n_corrected = n_dropped_ambiguous = n_dropped_no_board = 0
    for idxs in _cluster_detections(detections, cluster_dist):
        cluster = [detections[i] for i in idxs]
        board_votes = defaultdict(list)   # board_id -> [(code,x,y), ...]
        foreign = []                      # detections not on any board
        for det in cluster:
            code = det[0]
            b = CODE_TO_BOARD.get(code)
            if b is not None:
                board_votes[b].append(det)
            else:
                foreign.append(det)

        if not board_votes:
            n_dropped_no_board += len(cluster)
            continue

        board_id, members = max(board_votes.items(), key=lambda kv: len(kv[1]))
        if len(members) < min_board_votes:
            n_dropped_no_board += len(cluster)
            continue

        out.extend(members)
        # everything else in this cluster (foreign codes, or codes that voted
        # for a losing/minority board) is unexplained given this board
        unexplained = foreign + [d for b, ds in board_votes.items() if b != board_id for d in ds]
        if not unexplained:
            continue
        missing = set(BOARD_LAYOUT[board_id]) - {d[0] for d in members}
        if len(unexplained) == 1 and len(missing) == 1:
            code, x, y = unexplained[0]
            fixed_code = next(iter(missing))
            out.append((fixed_code, x, y))
            n_corrected += 1
        else:
            n_dropped_ambiguous += len(unexplained)

    if n_corrected or n_dropped_ambiguous or n_dropped_no_board:
        log(f"    screen: {n_corrected} corrected, {n_dropped_ambiguous} dropped "
            f"(ambiguous), {n_dropped_no_board} dropped (no board corroboration)")
    return out


# ======================================================================== #
#  I/O layer  (lazy backend imports live here)
# ======================================================================== #
def load_observations(calib_root, cct_n=12, cct_color="white"):
    """
    Walk the pass-major tree, detect CCT markers in every image, screen them
    against the known board layout (screen_detections), lift each surviving
    detection to a 3D point in that camera's own point cloud. Returns
    (obs_list, stop_names).

    Requires the backend-inspection `algorithms` package (open3d, cv2, CCTDecoder)
    to be importable — added to sys.path here, not at module import time.
    """
    from algorithms.my_pcd import MyPCD                     # noqa: E402
    from algorithms.CCTDecoder.cct_decode import CCT_extract  # noqa: E402

    calib_root = Path(calib_root)
    obs_list = []
    stop_names = set()

    pass_dirs = sorted(d for d in calib_root.iterdir() if d.is_dir())
    if not pass_dirs:
        raise FileNotFoundError(f"No pass_* folders under {calib_root}")

    for pass_dir in pass_dirs:
        pass_id = pass_dir.name
        stop_dirs = sorted(d for d in pass_dir.iterdir() if d.is_dir())
        for stop_dir in stop_dirs:
            stop = stop_dir.name
            stop_names.add(stop)
            for cam in ("left", "right"):
                cam_dir = stop_dir / cam
                if not (cam_dir / "Image.png").exists():
                    log(f"  skip missing {cam_dir}")
                    continue
                frame = MyPCD(cam_dir)
                _, _, raw = CCT_extract(frame.image, cct_n, color=cct_color,
                                         return_all_detections=True)
                screened = screen_detections(raw)
                n_lifted = 0
                for mid, px, py in screened:
                    xyz = frame.get_3dcoord_bilinear(float(px), float(py))
                    if xyz is None:
                        continue
                    obs_list.append(
                        Obs(pass_id, stop, "L" if cam == "left" else "R",
                            int(mid), np.asarray(xyz, dtype=float) * M_TO_MM))
                    n_lifted += 1
                log(f"  {pass_id}/{stop}/{cam}: {len(raw)} raw CCT, "
                    f"{len(screened)} after screening, {n_lifted} lifted to 3D")

    return obs_list, sorted(stop_names)


# ======================================================================== #
#  Initialization (closed-form Umeyama)
# ======================================================================== #
def _common(a_map, b_map):
    """Given two {id: xyz} dicts, return matched (src_ids, A(N,3), B(N,3))."""
    ids = sorted(set(a_map) & set(b_map))
    if not ids:
        return ids, np.empty((0, 3)), np.empty((0, 3))
    A = np.array([a_map[i] for i in ids])
    B = np.array([b_map[i] for i in ids])
    return ids, A, B


def _index_by(obs_list):
    """Nested dict: [pass_id][stop][cam] -> {marker_id: xyz}."""
    idx = defaultdict(lambda: defaultdict(lambda: defaultdict(dict)))
    for o in obs_list:
        idx[o.pass_id][o.stop][o.cam][o.marker_id] = o.xyz
    return idx


def init_lr(obs_list):
    """Init T_lr (right->left) by pooling all same-stop both-camera correspondences."""
    idx = _index_by(obs_list)
    R_pts, L_pts = [], []
    for pid, stops in idx.items():
        for stop, cams in stops.items():
            if "L" in cams and "R" in cams:
                _, Lc, Rc = _common(cams["L"], cams["R"])
                if len(Lc):
                    L_pts.append(Lc)
                    R_pts.append(Rc)
    if not R_pts:
        log("  WARNING: no same-stop both-camera markers -> T_lr init = identity")
        return np.eye(4)
    R_all = np.vstack(R_pts)
    L_all = np.vstack(L_pts)
    T = g.umeyama(R_all, L_all)          # right -> left
    log(f"  T_lr init from {len(R_all)} L/R pairs, "
        f"rmse={g.rigid_rmse(T, R_all, L_all):.3f} mm")
    return T


def init_stops(obs_list, stop_names, ref_stop):
    """
    Init each stop pose (left-cam -> reference) by chaining pairwise Umeyama between
    consecutive stops' left cameras (pooled over passes), then rebasing to ref_stop.
    """
    idx = _index_by(obs_list)

    def pooled_left(stop):
        """{id: xyz} pooled across passes for a stop's left cam (last pass wins on dup)."""
        out = {}
        for pid, stops in idx.items():
            if stop in stops and "L" in stops[stop]:
                out.update(stops[stop]["L"])
        return out

    T_abs = {stop_names[0]: np.eye(4)}
    for i in range(1, len(stop_names)):
        a, b = stop_names[i - 1], stop_names[i]
        # pair per pass so a marker's two 3D coords come from the SAME board pose
        Bp, Ap = [], []
        for pid, stops in idx.items():
            if a in stops and b in stops and "L" in stops[a] and "L" in stops[b]:
                _, Ac, Bc = _common(stops[a]["L"], stops[b]["L"])
                if len(Ac):
                    Ap.append(Ac)
                    Bp.append(Bc)
        if Bp:
            rel = g.umeyama(np.vstack(Bp), np.vstack(Ap))   # b -> a
            T_abs[b] = T_abs[a] @ rel
        else:
            log(f"  WARNING: no shared left markers between {a} and {b} -> chain gap, using identity step")
            T_abs[b] = T_abs[a].copy()

    base = g.invert(T_abs[ref_stop])
    return {s: base @ T_abs[s] for s in stop_names}


# ======================================================================== #
#  Bundle adjustment core  (pure numpy/scipy)
# ======================================================================== #
def _build_groups(obs_list):
    """(pass_id, marker_id) -> list of (stop, cam, xyz); keep multiview (>=2) only."""
    groups = defaultdict(list)
    for o in obs_list:
        groups[(o.pass_id, o.marker_id)].append((o.stop, o.cam, o.xyz))
    return {k: v for k, v in groups.items() if len(v) >= 2}


def _map_point(xyz, cam, T_s, T_lr):
    if cam == "L":
        return T_s[:3, :3] @ xyz + T_s[:3, 3]
    y = T_lr[:3, :3] @ xyz + T_lr[:3, 3]        # right -> left
    return T_s[:3, :3] @ y + T_s[:3, 3]         # left -> reference


def bundle_adjust(obs_list, stop_names, ref_stop, T_lr0=None, T_stops0=None,
                  verbose=True, robust_f_scale_mm=1.5):
    """
    Joint solve of T_lr and all stop poses. Global marker positions are profiled
    out analytically (residual = each mapped observation minus its group mean),
    so the only unknowns are the poses. Returns a result dict.

    Uses a soft_l1 robust loss (scale = robust_f_scale_mm) rather than plain
    least squares: screen_detections() catches most bad correspondences before
    they get here, but it can't catch everything (e.g. a lone false-positive
    that happens to land inside another board's cluster and get misattributed
    to a real missing slot). A robust loss down-weights whatever residual
    outliers slip through instead of letting a handful of them dominate the
    cost the way plain L2 does -- the same principle as the proven
    `filter_avg()` 1-sigma disparity trim from the sibling calibrate_lr
    project (algorithms/utils.py), adapted from a pairwise pre-filter to an
    in-solve robust loss since bundle_adjust's groups are n-way, not pairwise.
    Set to None for plain least squares (e.g. to match the old, pre-robust
    numeric behavior in tests).
    """
    stops = list(stop_names)
    if ref_stop not in stops:
        raise ValueError(f"reference stop {ref_stop!r} not among {stops}")
    free_stops = [s for s in stops if s != ref_stop]

    groups = _build_groups(obs_list)
    if not groups:
        raise RuntimeError("No multiview marker groups — cannot calibrate.")

    if T_lr0 is None:
        T_lr0 = init_lr(obs_list)
    if T_stops0 is None:
        T_stops0 = init_stops(obs_list, stops, ref_stop)

    # pack parameters: [T_lr(6), free_stop_0(6), free_stop_1(6), ...]
    p0 = np.concatenate([g.mat_to_vec(T_lr0)] +
                        [g.mat_to_vec(T_stops0[s]) for s in free_stops])

    def unpack(p):
        T_lr = g.vec_to_mat(p[0:6])
        Ts = {ref_stop: np.eye(4)}
        for i, s in enumerate(free_stops):
            Ts[s] = g.vec_to_mat(p[6 + 6 * i: 12 + 6 * i])
        return T_lr, Ts

    # precompute group observation tuples for speed
    group_obs = list(groups.values())

    def residuals(p):
        T_lr, Ts = unpack(p)
        chunks = []
        for obs in group_obs:
            mapped = np.array([_map_point(xyz, cam, Ts[s], T_lr)
                               for (s, cam, xyz) in obs])
            chunks.append((mapped - mapped.mean(axis=0)).ravel())
        return np.concatenate(chunks)

    r0 = residuals(p0)
    log(f"  BA: {len(groups)} groups, {sum(len(v) for v in group_obs)} obs, "
        f"{len(p0)} params, {len(r0)} residuals; "
        f"init RMS={_rms(r0):.3f} mm")

    robust_kwargs = {}
    if robust_f_scale_mm is not None:
        # residuals() returns flat per-axis (x,y,z) components, not per-point
        # 3D magnitudes, so f_scale is a per-component mm threshold -- inlier
        # groups here run ~0.3-0.6mm per axis, so 1.5mm comfortably separates
        # them from the tens-of-mm outliers a bad correspondence produces.
        robust_kwargs = {"loss": "soft_l1", "f_scale": robust_f_scale_mm}
    sol = least_squares(residuals, p0, method="trf",
                        xtol=1e-12, ftol=1e-12, gtol=1e-12,
                        verbose=2 if verbose else 0, **robust_kwargs)
    T_lr, Ts = unpack(sol.x)
    final = residuals(sol.x)
    log(f"  BA done: final RMS={_rms(final):.3f} mm  "
        f"(scipy status {sol.status})")

    stats = residual_report(obs_list, stops, ref_stop, T_lr, Ts)
    return {
        "T_lr": T_lr,
        "T_stops": Ts,
        "stop_names": stops,
        "reference_stop": ref_stop,
        "final_rms_mm": _rms(final),
        "stats": stats,
    }


def _rms(vec):
    """RMS of Euclidean norms of (N,3) vectors, in whatever unit `vec` is in."""
    vec = np.asarray(vec).reshape(-1, 3)
    return float(np.sqrt(np.mean(np.sum(vec ** 2, axis=1)))) if len(vec) else 0.0


def residual_report(obs_list, stop_names, ref_stop, T_lr, Ts):
    """Per-stop / per-pass / L-R residual RMS (mm) after a solve."""
    groups = _build_groups(obs_list)
    per_stop = defaultdict(list)
    per_pass = defaultdict(list)
    lr_res = []           # residual of right obs at stops that also have left of same id
    all_res = []
    for (pid, mid), obs in groups.items():
        mapped = np.array([_map_point(xyz, cam, Ts[s], T_lr) for (s, cam, xyz) in obs])
        mean = mapped.mean(axis=0)
        has_L = any(c == "L" for (_, c, _) in obs)
        has_R = any(c == "R" for (_, c, _) in obs)
        for (s, cam, _), m in zip(obs, mapped):
            e = np.linalg.norm(m - mean)
            per_stop[s].append(e)
            per_pass[pid].append(e)
            all_res.append(e)
            if cam == "R" and has_L and has_R:
                lr_res.append(e)

    def summ(d):
        return {k: {"rms_mm": float(np.sqrt(np.mean(np.square(v)))), "n": len(v)}
                for k, v in sorted(d.items())}

    return {
        "overall_rms_mm": float(np.sqrt(np.mean(np.square(all_res)))) if all_res else 0.0,
        "n_residual_points": len(all_res),
        "per_stop": summ(per_stop),
        "per_pass": summ(per_pass),
        "lr_rms_mm": float(np.sqrt(np.mean(np.square(lr_res)))) if lr_res else None,
        "n_groups": len(groups),
    }


# ======================================================================== #
#  pose(x) sag model + output
# ======================================================================== #
def fit_and_eval_posex(result, stop_x, operational_stops, model_kind="poly",
                       poly_deg=3, crosscheck_tol_mm=2.0, crosscheck_tol_deg=0.05,
                       prefer_direct=True, match_tol_mm=1.0):
    """
    Fit pose(x) over captured stops, evaluate at operational stops.
    Returns (model, traj_ext, crosscheck) where traj_ext = {op_name: 4x4}.

    `prefer_direct` (default True): when an operational stop sits at the same
    rail position as a stop that was actually calibrated, emit that stop's
    measured bundle-adjusted pose rather than the smoothed pose(x) value.

    This matters when calibration and inspection share stops -- the common case.
    pose(x) exists to interpolate a dense calibration grid onto operational
    positions that were never visited; where a position WAS visited, the direct
    estimate is a measurement and the model value is a fit to it. A low-order
    polynomial through few stops will not reproduce per-stop rotation exactly,
    so preferring the model there discards real information and injects the
    residual as pointing error. The cross-check below reports the difference
    either way.
    """
    stops = result["stop_names"]
    xs = np.array([stop_x[s] for s in stops], dtype=float)
    poses = [result["T_stops"][s] for s in stops]
    model = g.fit_pose_x(xs, poses, kind=model_kind, poly_deg=poly_deg)

    def _match(x_op):
        """Captured stop at this rail position, if any."""
        near = [s for s in stops if abs(stop_x[s] - float(x_op)) <= match_tol_mm]
        return near[0] if near else None

    traj_ext = {}
    sources = {}
    for op_name, x_op in operational_stops.items():
        s = _match(x_op) if prefer_direct else None
        if s is not None:
            traj_ext[op_name] = result["T_stops"][s]
            sources[op_name] = f"direct({s})"
        else:
            traj_ext[op_name] = model.eval_pose(float(x_op))
            sources[op_name] = "model"
    n_direct = sum(1 for v in sources.values() if v.startswith("direct"))
    log(f"  traj_ext: {n_direct}/{len(traj_ext)} stops from direct BA poses, "
        f"{len(traj_ext) - n_direct} interpolated from pose(x)")

    # Cross-check: where an operational x coincides with a captured stop, compare
    # the measured pose against what the model would have produced. With
    # prefer_direct this no longer affects the output -- it is a diagnostic of
    # how well the smooth model describes the rig.
    crosscheck = []
    for op_name, x_op in operational_stops.items():
        s = _match(x_op)
        if s is None:
            continue
        T_direct = result["T_stops"][s]
        T_model = model.eval_pose(float(x_op))
        dt = np.linalg.norm(T_direct[:3, 3] - T_model[:3, 3])
        dR = np.degrees(np.linalg.norm(
            Rotation.from_matrix(T_direct[:3, :3] @ T_model[:3, :3].T).as_rotvec()))
        ok = (dt <= crosscheck_tol_mm) and (dR <= crosscheck_tol_deg)
        crosscheck.append({"op": op_name, "stop": s, "dt_mm": dt, "dR_deg": dR,
                           "ok": ok, "used": sources[op_name]})
        if not ok:
            level = "note" if sources[op_name].startswith("direct") else "WARN"
            log(f"  CROSSCHECK {level} {op_name}~{s}: dt={dt:.2f}mm dR={dR:.3f}deg "
                f"exceeds tol -> pose(x) does not reproduce this stop"
                + ("; using the direct pose, so the output is unaffected"
                   if sources[op_name].startswith("direct")
                   else "; this stop IS interpolated, so the error is in the output"))
    return model, traj_ext, crosscheck


def plot_sag(result, stop_x, out_png):
    """Sag plot: 6 DOF vs rail x with fitted curve. Lazy matplotlib import."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        log("  matplotlib not available -> skipping sag_curve.png "
            "(run in the beta3d/reborn env to get the plot)")
        return None

    stops = result["stop_names"]
    xs = np.array([stop_x[s] for s in stops], dtype=float)
    vecs = np.array([g.mat_to_vec(result["T_stops"][s]) for s in stops])
    model = g.fit_pose_x(xs, [result["T_stops"][s] for s in stops])
    xg = np.linspace(xs.min(), xs.max(), 200)
    vg = model.eval_vec(xg)

    # T_stops translation is already mm here (load_observations converts real
    # point-cloud xyz meters->mm at ingestion; see M_TO_MM). Rotation (rotvec,
    # radians) and stop_x (from the PLC, mm) need no conversion.
    labels = ["rotvec_x (rad)", "rotvec_y (rad)", "rotvec_z (rad)",
              "t_x (mm)", "t_y (mm)", "t_z (mm)"]
    fig, axes = plt.subplots(2, 3, figsize=(15, 8))
    for dof, ax in enumerate(axes.ravel()):
        ax.plot(xg, vg[:, dof], "-", color="#ff8800", lw=2, label="pose(x) fit")
        ax.plot(xs, vecs[:, dof], "o", color="#0066cc", label="BA per-stop")
        ax.set_title(labels[dof])
        ax.set_xlabel("rail position x (mm)")
        ax.grid(alpha=0.3)
        if dof == 0:
            ax.legend(fontsize=8)
    fig.suptitle("Rig pose vs rail position (sag profile)", fontsize=14)
    fig.tight_layout()
    fig.savefig(out_png, dpi=130)
    plt.close(fig)
    log(f"  wrote {out_png}")
    return out_png


def _jsonable(o):
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, dict):
        return {k: _jsonable(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_jsonable(v) for v in o]
    # np.generic covers every numpy scalar type (np.floating, np.integer, and
    # crucially np.bool_, which the previous np.floating/np.integer-only check
    # missed). crosscheck's "ok" is `(dt <= tol) and (dR <= tol)` on
    # np.linalg.norm() floats, i.e. np.bool_, not a native bool -- that silently
    # broke json.dump on every run, hidden until now behind an earlier crash
    # (write_outputs used to run after plot_sag, which failed first on a
    # missing out_dir).
    if isinstance(o, np.generic):
        return o.item()
    return o


def _mm_pose_to_m(T):
    """Copy of a 4x4 pose with its translation column converted mm -> meters."""
    T = np.array(T, dtype=float, copy=True)
    T[:3, 3] /= M_TO_MM
    return T


def write_outputs(result, traj_ext, crosscheck, out_dir):
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    # result["T_lr"]/traj_ext are mm internally (see M_TO_MM). The runtime
    # consumer (combine_frames_extrinsic) applies these to point clouds loaded
    # via MyPCD, which are natively meters -- so only the two pickles consumed
    # by that runtime get converted back; calib_report.json below stays mm.
    with open(out_dir / "left_right_ext.pkl", "wb") as f:
        pickle.dump(_mm_pose_to_m(result["T_lr"]), f)
    with open(out_dir / "cam_traj_ext.pkl", "wb") as f:
        pickle.dump({k: _mm_pose_to_m(v) for k, v in traj_ext.items()}, f)

    report = {
        "reference_stop": result["reference_stop"],
        "final_rms_mm": result["final_rms_mm"],
        "stats": result["stats"],
        "T_lr": result["T_lr"],
        "captured_stop_poses": {s: result["T_stops"][s] for s in result["stop_names"]},
        "operational_traj_ext": traj_ext,
        "crosscheck": crosscheck,
    }
    with open(out_dir / "calib_report.json", "w", encoding="utf-8") as f:
        json.dump(_jsonable(report), f, indent=2, ensure_ascii=False)
    log(f"  wrote left_right_ext.pkl, cam_traj_ext.pkl, calib_report.json to {out_dir}")


# ======================================================================== #
#  CLI
# ======================================================================== #
def run(config):
    calib_root = config["calib_root"]
    out_dir = config.get("out_dir", str(Path(calib_root) / "calib_out"))
    ref_stop = config.get("reference_stop")
    cct_n = config.get("cct_n", 12)

    log(f"loading observations from {calib_root} ...")
    obs_list, stop_names = load_observations(calib_root, cct_n=cct_n)
    log(f"stops found: {stop_names}")
    if ref_stop is None:
        ref_stop = stop_names[0]
        log(f"reference_stop defaulting to {ref_stop}")

    result = bundle_adjust(obs_list, stop_names, ref_stop)
    log(f"overall RMS {result['stats']['overall_rms_mm']:.3f} mm, "
        f"L-R RMS {result['stats']['lr_rms_mm']}")

    # rail positions per captured stop (fallback to ordinal with a warning)
    stop_x = config.get("stop_x")
    if not stop_x:
        stop_x = {s: float(i) for i, s in enumerate(stop_names)}
        log("WARNING: no stop_x given -> using ordinal positions; sag axis is in "
            "'stop units', not mm. Provide stop_x for a physical sag curve.")
    stop_x = {k: float(v) for k, v in stop_x.items()}

    operational = config.get("operational_stops")
    if not operational:
        operational = {s: stop_x[s] for s in stop_names}
        log("no operational_stops given -> emitting captured stops as-is")
    operational = {k: float(v) for k, v in operational.items()}

    model_cfg = config.get("model", {})
    model, traj_ext, crosscheck = fit_and_eval_posex(
        result, stop_x, operational,
        model_kind=model_cfg.get("kind", "poly"),
        poly_deg=model_cfg.get("poly_deg", 3),
        crosscheck_tol_mm=config.get("crosscheck_tol_mm", 2.0),
        crosscheck_tol_deg=config.get("crosscheck_tol_deg", 0.05),
        prefer_direct=config.get("prefer_direct", True))

    # Create out_dir HERE, not in write_outputs: plot_sag writes into it first,
    # and a missing directory there threw away a completed bundle adjustment.
    Path(out_dir).mkdir(parents=True, exist_ok=True)

    # Write the real artifacts BEFORE plotting, and never let a plotting problem
    # lose them. The BA can be an hour of work; sag_curve.png is a diagnostic.
    write_outputs(result, traj_ext, crosscheck, out_dir)
    try:
        plot_sag(result, stop_x, str(Path(out_dir) / "sag_curve.png"))
    except Exception as e:
        log(f"  WARNING: sag plot failed ({e}); pickles and report were still written")
    log("done.")
    return result, traj_ext


def main():
    ap = argparse.ArgumentParser(description="Joint L-R + trajectory rig calibration.")
    ap.add_argument("--config", help="JSON config file", default=None)
    ap.add_argument("--calib-root", help="pass-major capture root (overrides config)")
    ap.add_argument("--out-dir", help="output directory (overrides config)")
    ap.add_argument("--reference-stop", help="stop name to fix as reference")
    args = ap.parse_args()

    config = {}
    if args.config:
        with open(args.config, encoding="utf-8") as f:
            config = json.load(f)
    if args.calib_root:
        config["calib_root"] = args.calib_root
    if args.out_dir:
        config["out_dir"] = args.out_dir
    if args.reference_stop:
        config["reference_stop"] = args.reference_stop
    if "calib_root" not in config:
        ap.error("calib_root required (via --calib-root or config)")

    run(config)


if __name__ == "__main__":
    main()
