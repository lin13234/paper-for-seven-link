import numpy as np

from planar_fourbar_dh.fourbar_dh import (
    extract_fourbar_geometry,
    forward_kinematics,
    solve_explicit,
)


def test_zero_configuration_is_closed():
    """默认 DH 初始姿态必须严格满足七杆闭环。"""
    transforms = forward_kinematics(np.zeros(7))
    np.testing.assert_allclose(transforms[-1], np.eye(4), atol=1e-12)


def test_active_axes_and_link_lengths_at_zero():
    """关节 2/3/5/7 轴线平行，四杆长度与初始几何一致。"""
    geometry = extract_fourbar_geometry()
    expected = np.array([np.sqrt(41.0) / 2.0, 5.0, np.sqrt(41.0) / 2.0, 1.0])
    np.testing.assert_allclose(geometry.lengths, expected, atol=1e-12)
    assert geometry.axes_are_parallel


def test_explicit_solution_recovers_zero_configuration():
    """输入角位于初始值时，解析解必须恢复原七杆关节变量。"""
    solution = solve_explicit(np.array([0.0]))
    np.testing.assert_allclose(solution.q[0], np.zeros(7), atol=1e-11)
    np.testing.assert_allclose(solution.points[0, 2], [-2.0, -2.5], atol=1e-11)


def test_full_revolution_satisfies_seven_joint_dh_closure():
    """完整输入周期内，显式解必须同时满足几何闭环和 DH 闭环。"""
    q2 = np.linspace(0.0, 2.0 * np.pi, 361)
    solution = solve_explicit(q2)
    assert np.all(solution.reachable)
    assert np.max(solution.position_error) < 1e-10
    assert np.max(solution.rotation_error) < 1e-10


def test_locked_joints_stay_at_initial_values_and_angles_are_continuous():
    """锁死关节保持初始角，活动关节曲线不得出现 atan2 人为跳变。"""
    solution = solve_explicit(np.linspace(0.0, 2.0 * np.pi, 721))
    expected_locked = np.broadcast_to(solution.q0[[1, 3, 5]], (721, 3))
    np.testing.assert_allclose(solution.q[:, [1, 3, 5]], expected_locked, atol=0.0)
    assert np.max(np.abs(np.diff(solution.q[:, [2, 4, 6]], axis=0))) < 0.2
