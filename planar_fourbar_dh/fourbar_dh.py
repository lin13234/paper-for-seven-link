"""基于改进 DH 参数的 2-4-6-1 平面四杆显式解。

本模块严格沿用用户原脚本中的变换矩阵。默认初始关节变量为零，
其中第 3、6 个 DH 角仍包含参数表给出的 pi 固定偏置。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike, NDArray


FloatArray = NDArray[np.float64]
ACTIVE = np.array([0, 2, 4, 6], dtype=int)
LOCKED = np.array([1, 3, 5], dtype=int)


def dh_parameters(l: float = 1.0) -> FloatArray:
    """返回 [alpha_(i-1), a_(i-1), d_i, theta_offset] 参数表。"""
    return np.array(
        [
            (0.0, l, 0.0, 0.0),
            (np.pi / 2.0, 2.0 * l, 2.5 * l, 0.0),
            (np.pi / 2.0, 0.0, 0.0, np.pi),
            (np.pi / 2.0, 2.5 * l, 0.0, 0.0),
            (np.pi / 2.0, 2.5 * l, 0.0, 0.0),
            (np.pi / 2.0, 0.0, 2.5 * l, np.pi),
            (np.pi / 2.0, 2.0 * l, 0.0, 0.0),
        ],
        dtype=float,
    )


def dh_transform(alpha: float, a: float, d: float, theta: float) -> FloatArray:
    """构造与原脚本完全一致的改进 DH 齐次变换矩阵。"""
    ca, sa = np.cos(alpha), np.sin(alpha)
    ct, st = np.cos(theta), np.sin(theta)
    return np.array(
        [
            [ct, -st, 0.0, a],
            [st * ca, ct * ca, -sa, -d * sa],
            [st * sa, ct * sa, ca, d * ca],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


def forward_kinematics(q: ArrayLike, l: float = 1.0) -> list[FloatArray]:
    """计算从基坐标系到每一级连杆的累计变换，列表首项为单位阵。"""
    q_array = np.asarray(q, dtype=float)
    if q_array.shape != (7,):
        raise ValueError("q 必须是长度为 7 的一维数组。")

    transforms = [np.eye(4, dtype=float)]
    current = transforms[0]
    for qi, (alpha, a, d, offset) in zip(q_array, dh_parameters(l)):
        current = current @ dh_transform(alpha, a, d, qi + offset)
        transforms.append(current.copy())
    return transforms


def joint_axis_geometry(q: ArrayLike, l: float = 1.0) -> tuple[FloatArray, FloatArray]:
    """计算七个转动关节在基坐标系中的轴上点和轴线方向。

    原矩阵先施加固定的 x 向平移与 x 向扭转，再绕新的 z 轴转动。
    因而关节轴不是简单的累计坐标系旧 z 轴。
    """
    q_array = np.asarray(q, dtype=float)
    params = dh_parameters(l)
    transforms = forward_kinematics(q_array, l)
    points = np.zeros((7, 3), dtype=float)
    axes = np.zeros((7, 3), dtype=float)

    for i, (alpha, a, _d, _offset) in enumerate(params):
        previous = transforms[i]
        rotation = previous[:3, :3]
        points[i] = (previous @ np.array([a, 0.0, 0.0, 1.0]))[:3]
        axes[i] = rotation @ np.array([0.0, -np.sin(alpha), np.cos(alpha)])
        axes[i] /= np.linalg.norm(axes[i])
    return points, axes


@dataclass(frozen=True)
class FourBarGeometry:
    """由初始七杆姿态提取的平面四杆几何。"""

    q0: FloatArray
    points_3d: FloatArray
    points_2d: FloatArray
    axes: FloatArray
    axis_signs: FloatArray
    origin: FloatArray
    basis_x: FloatArray
    basis_y: FloatArray
    normal: FloatArray
    lengths: FloatArray
    axes_are_parallel: bool


@dataclass(frozen=True)
class FourBarSolution:
    """显式求解得到的关节角、轴心位置及完整 DH 闭环误差。"""

    q0: FloatArray
    q: FloatArray
    points: FloatArray
    reachable: NDArray[np.bool_]
    position_error: FloatArray
    rotation_error: FloatArray
    branch_sign: int
    geometry: FourBarGeometry


def extract_fourbar_geometry(
    l: float = 1.0,
    q0: ArrayLike | None = None,
    tolerance: float = 1e-9,
) -> FourBarGeometry:
    """从初始姿态提取四根活动轴在公共平面内的轴心和杆长。"""
    initial = np.zeros(7, dtype=float) if q0 is None else np.asarray(q0, dtype=float)
    if initial.shape != (7,):
        raise ValueError("q0 必须是长度为 7 的一维数组。")

    closure = forward_kinematics(initial, l)[-1]
    if np.linalg.norm(closure - np.eye(4)) > tolerance:
        raise ValueError("给定 q0 不满足完整七杆 DH 闭环。")

    all_points, all_axes = joint_axis_geometry(initial, l)
    points_3d = all_points[ACTIVE]
    axes = all_axes[ACTIVE]
    normal = axes[0].copy()
    signs = axes @ normal
    parallel = bool(np.all(np.abs(np.abs(signs) - 1.0) < tolerance))
    if not parallel:
        raise ValueError("初始姿态下关节 2、4、6、1 的轴线不平行。")

    # 以 P1 为二维原点、P1->P2 为 x 轴，法向按关节 2 正方向确定。
    origin = points_3d[3].copy()
    basis_x = points_3d[0] - origin
    basis_x -= np.dot(basis_x, normal) * normal
    basis_x /= np.linalg.norm(basis_x)
    basis_y = np.cross(normal, basis_x)
    basis_y /= np.linalg.norm(basis_y)

    relative = points_3d - origin
    points_2d = np.column_stack((relative @ basis_x, relative @ basis_y))
    lengths = np.linalg.norm(
        np.roll(points_2d, -1, axis=0) - points_2d,
        axis=1,
    )
    return FourBarGeometry(
        q0=initial.copy(),
        points_3d=points_3d,
        points_2d=points_2d,
        axes=axes,
        axis_signs=np.sign(signs),
        origin=origin,
        basis_x=basis_x,
        basis_y=basis_y,
        normal=normal,
        lengths=lengths,
        axes_are_parallel=parallel,
    )


def _wrap_to_pi(angle: FloatArray | float) -> FloatArray:
    """把角度包裹到 [-pi, pi)，仅用于比较候选支路。"""
    value = np.asarray(angle, dtype=float)
    return (value + np.pi) % (2.0 * np.pi) - np.pi


def _oriented_angle(vector: FloatArray) -> float:
    """返回二维向量相对 +x 方向的有向角。"""
    return float(np.arctan2(vector[1], vector[0]))


def _circle_point(
    moving_center: FloatArray,
    moving_radius: float,
    fixed_center: FloatArray,
    fixed_radius: float,
    sign: int,
    tolerance: float,
) -> tuple[FloatArray, bool]:
    """用两圆交点显式公式计算 P6，sign 对应两条装配支路。"""
    delta = fixed_center - moving_center
    distance = float(np.linalg.norm(delta))
    if distance <= tolerance:
        return np.full(2, np.nan), False

    unit = delta / distance
    perpendicular = np.array([-unit[1], unit[0]])
    x = (moving_radius**2 - fixed_radius**2 + distance**2) / (2.0 * distance)
    h2 = moving_radius**2 - x**2
    if h2 < -tolerance:
        return np.full(2, np.nan), False

    height = np.sqrt(max(h2, 0.0))
    return moving_center + x * unit + sign * height * perpendicular, True


def _select_initial_branch(geometry: FourBarGeometry, tolerance: float) -> int:
    """选择在输入初值处通过原始 P6 的圆交点符号。"""
    p4 = geometry.points_2d[1]
    p6_initial = geometry.points_2d[2]
    p1 = geometry.points_2d[3]
    candidates = {}
    for sign in (-1, 1):
        point, valid = _circle_point(
            p4,
            geometry.lengths[1],
            p1,
            geometry.lengths[2],
            sign,
            tolerance,
        )
        candidates[sign] = np.linalg.norm(point - p6_initial) if valid else np.inf
    return min(candidates, key=candidates.get)


def _closure_errors(q: FloatArray, l: float) -> tuple[float, float]:
    """计算完整七杆末端相对单位阵的位置误差和旋转角误差。"""
    end = forward_kinematics(q, l)[-1]
    position_error = float(np.linalg.norm(end[:3, 3]))
    cosine = np.clip((np.trace(end[:3, :3]) - 1.0) / 2.0, -1.0, 1.0)
    rotation_error = float(np.arccos(cosine))
    return position_error, rotation_error


def solve_explicit(
    q2_values: ArrayLike,
    l: float = 1.0,
    q0: ArrayLike | None = None,
    branch: int | None = None,
    tolerance: float = 1e-10,
) -> FourBarSolution:
    """求同一装配支路上 q4(q2)、q6(q2)、q1(q2) 的显式解。

    参数 q2_values 是原始 DH 关节变量 q2，而非相对输入增量。
    关节 3、5、7 始终保持 q0 中的初始值。
    """
    inputs = np.atleast_1d(np.asarray(q2_values, dtype=float))
    if inputs.ndim != 1:
        raise ValueError("q2_values 必须是一维数组。")

    geometry = extract_fourbar_geometry(l=l, q0=q0, tolerance=max(tolerance, 1e-9))
    sign = _select_initial_branch(geometry, tolerance) if branch is None else int(branch)
    if sign not in (-1, 1):
        raise ValueError("branch 只能取 -1 或 +1。")

    p2_initial, p4_initial, p6_initial, p1 = geometry.points_2d
    initial_vectors = np.array(
        [p4_initial - p2_initial, p6_initial - p4_initial, p1 - p6_initial]
    )
    initial_angles = np.array([_oriented_angle(v) for v in initial_vectors])
    sigma2, sigma4, sigma6, sigma1 = geometry.axis_signs

    q = np.broadcast_to(geometry.q0, (len(inputs), 7)).copy()
    points = np.full((len(inputs), 4, 2), np.nan, dtype=float)
    reachable = np.zeros(len(inputs), dtype=bool)
    position_error = np.full(len(inputs), np.nan, dtype=float)
    rotation_error = np.full(len(inputs), np.nan, dtype=float)
    raw_deltas = np.full((len(inputs), 3), np.nan, dtype=float)

    for row, q2 in enumerate(inputs):
        delta_q2 = q2 - geometry.q0[0]
        angle = sigma2 * delta_q2
        c, s = np.cos(angle), np.sin(angle)
        rotation_2d = np.array([[c, -s], [s, c]])
        p4 = p2_initial + rotation_2d @ (p4_initial - p2_initial)
        p6, valid = _circle_point(
            p4,
            geometry.lengths[1],
            p1,
            geometry.lengths[2],
            sign,
            tolerance,
        )
        points[row] = np.array([p2_initial, p4, p6, p1])
        if not valid:
            continue

        current_angles = np.array(
            [
                _oriented_angle(p4 - p2_initial),
                _oriented_angle(p6 - p4),
                _oriented_angle(p1 - p6),
            ]
        )
        raw_deltas[row] = _wrap_to_pi(current_angles - initial_angles)
        reachable[row] = True

    # 沿输入序列连续展开杆件方向变化，再映射为各 DH 关节增量。
    valid_rows = np.flatnonzero(reachable)
    if valid_rows.size:
        angle_deltas = np.unwrap(raw_deltas[valid_rows], axis=0)
        for local_row, row in enumerate(valid_rows):
            delta_q2 = inputs[row] - geometry.q0[0]
            delta_phi46 = angle_deltas[local_row, 1]
            delta_phi61 = angle_deltas[local_row, 2]
            delta_q4 = (delta_phi46 - sigma2 * delta_q2) / sigma4
            delta_q6 = (
                delta_phi61
                - sigma2 * delta_q2
                - sigma4 * delta_q4
            ) / sigma6
            delta_q1 = -(
                sigma2 * delta_q2 + sigma4 * delta_q4 + sigma6 * delta_q6
            ) / sigma1

            q[row, 0] = inputs[row]
            q[row, 2] = geometry.q0[2] + delta_q4
            q[row, 4] = geometry.q0[4] + delta_q6
            q[row, 6] = geometry.q0[6] + delta_q1
            position_error[row], rotation_error[row] = _closure_errors(q[row], l)

    return FourBarSolution(
        q0=geometry.q0.copy(),
        q=q,
        points=points,
        reachable=reachable,
        position_error=position_error,
        rotation_error=rotation_error,
        branch_sign=sign,
        geometry=geometry,
    )
