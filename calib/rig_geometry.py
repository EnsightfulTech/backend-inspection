"""
rig_geometry.py — reusable rigid-transform math for the gantry rig calibration.

Pure numpy + scipy (no open3d / no camera SDK), so it is unit-testable in isolation.

Conventions used throughout the calibration:
  * A pose is a 4x4 homogeneous matrix T that maps a point X (3,) as  T @ [X;1].
  * `umeyama(src, dst)` returns T such that  dst ~= T @ src  (rigid, no scale).
  * A pose is parametrized for the optimizer as a 6-vector [rotvec(3), t(3)]
    where R = Rotation.from_rotvec(rotvec). This is a clean local parametrization
    (NOT twist/se3-exp coordinates) — rotation and translation are independent,
    which is all the least-squares refinement needs.

This module also carries the *correct* rigid-fit that fixes the historical bug in
`optimize.py` / `my_pcd.py::estimate_RT_CCT_optimize`, where the translation was
reconstructed as `t = dt + u1 - u0` instead of `t = dt + u1 - R @ u0`.
"""

import numpy as np
from scipy.spatial.transform import Rotation


# --------------------------------------------------------------------------- #
# Basic pose helpers
# --------------------------------------------------------------------------- #
def identity_pose() -> np.ndarray:
    return np.eye(4)


def invert(T: np.ndarray) -> np.ndarray:
    """Inverse of a 4x4 rigid transform."""
    R = T[:3, :3]
    t = T[:3, 3]
    Ti = np.eye(4)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ t
    return Ti


def apply(T: np.ndarray, pts: np.ndarray) -> np.ndarray:
    """Apply pose T to an (N,3) array of points -> (N,3)."""
    pts = np.asarray(pts, dtype=float)
    return pts @ T[:3, :3].T + T[:3, 3]


def compose(*mats: np.ndarray) -> np.ndarray:
    """Left-to-right matrix product: compose(A, B, C) == A @ B @ C."""
    out = np.eye(4)
    for m in mats:
        out = out @ m
    return out


# --------------------------------------------------------------------------- #
# 6-vector <-> matrix (optimizer parametrization)
# --------------------------------------------------------------------------- #
def mat_to_vec(T: np.ndarray) -> np.ndarray:
    """4x4 pose -> [rotvec(3), t(3)]."""
    rotvec = Rotation.from_matrix(T[:3, :3]).as_rotvec()
    return np.concatenate([rotvec, T[:3, 3]])


def vec_to_mat(v: np.ndarray) -> np.ndarray:
    """[rotvec(3), t(3)] -> 4x4 pose."""
    v = np.asarray(v, dtype=float)
    T = np.eye(4)
    T[:3, :3] = Rotation.from_rotvec(v[:3]).as_matrix()
    T[:3, 3] = v[3:]
    return T


# --------------------------------------------------------------------------- #
# Closed-form rigid fit (Umeyama / Kabsch, no scale)
# --------------------------------------------------------------------------- #
def umeyama(src: np.ndarray, dst: np.ndarray) -> np.ndarray:
    """
    Least-squares rigid transform T (4x4) with  dst ~= T @ src.

    src, dst : (N,3) corresponding points. Requires N >= 3 non-degenerate points.
    Returns the 4x4 pose. Reflection is prevented via the standard det-sign fix.
    """
    src = np.asarray(src, dtype=float)
    dst = np.asarray(dst, dtype=float)
    if src.shape != dst.shape or src.ndim != 2 or src.shape[1] != 3:
        raise ValueError(f"umeyama expects matching (N,3) arrays, got {src.shape}/{dst.shape}")
    if src.shape[0] < 3:
        raise ValueError(f"umeyama needs >=3 points, got {src.shape[0]}")

    mu_src = src.mean(axis=0)
    mu_dst = dst.mean(axis=0)
    X = src - mu_src
    Y = dst - mu_dst

    H = X.T @ Y                      # 3x3 cross-covariance
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    D = np.diag([1.0, 1.0, d])
    R = Vt.T @ D @ U.T               # maps src -> dst
    t = mu_dst - R @ mu_src          # <-- the R @ mu_src the old code dropped

    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def rigid_rmse(T: np.ndarray, src: np.ndarray, dst: np.ndarray) -> float:
    """RMS residual (same units as points) of  dst vs T @ src."""
    pred = apply(T, src)
    return float(np.sqrt(np.mean(np.sum((pred - np.asarray(dst)) ** 2, axis=1))))


# --------------------------------------------------------------------------- #
# pose(x) continuous model — one smooth curve per DOF vs rail position
# --------------------------------------------------------------------------- #
class PoseXModel:
    """
    Smooth model of pose-as-a-function-of-rail-position.

    Each of the 6 DOF (rotvec_x, rotvec_y, rotvec_z, t_x, t_y, t_z) of the stop
    pose (relative to the reference stop) is fit as a 1-D smooth curve of the rail
    coordinate x. Rotation is kept as a rotvec: for a sagging rail the angles are
    small (<~1 deg) so the rotvec components vary smoothly with no wrap issues.

    Backend is a low-order polynomial by default (beam-deflection shape); a
    smoothing spline is available when there are enough distinct samples.
    """

    def __init__(self, x_samples, pose_vecs, kind="poly", poly_deg=3, spline_s=None):
        x = np.asarray(x_samples, dtype=float)
        V = np.asarray(pose_vecs, dtype=float)      # (N, 6)
        if V.ndim != 2 or V.shape[1] != 6:
            raise ValueError(f"pose_vecs must be (N,6), got {V.shape}")
        order = np.argsort(x)
        self.x = x[order]
        self.V = V[order]
        self.kind = kind
        self.poly_deg = poly_deg
        self._models = []

        n = len(self.x)
        for dof in range(6):
            y = self.V[:, dof]
            if kind == "spline":
                from scipy.interpolate import UnivariateSpline
                # spline degree capped by sample count
                k = min(3, max(1, n - 1))
                s = spline_s if spline_s is not None else len(y) * np.var(y) * 1e-3
                self._models.append(("spline", UnivariateSpline(self.x, y, k=k, s=s)))
            else:
                deg = min(poly_deg, n - 1)
                self._models.append(("poly", np.polynomial.Polynomial.fit(self.x, y, deg)))

    def eval_vec(self, x_query) -> np.ndarray:
        """Return the 6-vector pose parametrization at rail position(s) x_query."""
        xq = np.atleast_1d(np.asarray(x_query, dtype=float))
        out = np.zeros((len(xq), 6))
        for dof, (kind, m) in enumerate(self._models):
            out[:, dof] = m(xq)
        return out[0] if np.isscalar(x_query) or np.ndim(x_query) == 0 else out

    def eval_pose(self, x_query) -> np.ndarray:
        """Return the 4x4 pose at a single rail position x_query."""
        return vec_to_mat(self.eval_vec(float(x_query)))

    def residuals(self) -> np.ndarray:
        """(N,6) fit residuals at the sample points (per DOF)."""
        pred = np.array([self.eval_vec(xi) for xi in self.x])
        return self.V - pred


def fit_pose_x(x_samples, poses, kind="poly", poly_deg=3, spline_s=None) -> PoseXModel:
    """
    Convenience wrapper: `poses` may be a list of 4x4 matrices or an (N,6) array
    of pose vectors. Returns a fitted PoseXModel.
    """
    poses = list(poses)
    if np.ndim(poses[0]) == 2:      # list of 4x4
        vecs = np.array([mat_to_vec(T) for T in poses])
    else:
        vecs = np.asarray(poses, dtype=float)
    return PoseXModel(x_samples, vecs, kind=kind, poly_deg=poly_deg, spline_s=spline_s)
