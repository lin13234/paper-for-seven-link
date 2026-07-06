from __future__ import annotations

import importlib.util
import sys
from pathlib import Path


def load_module():
    """加载 Joint2 解析分支跟踪脚本。"""
    module_path = Path(r"E:\hub\symbolic_mdh_joint2\trace_joint2_branch.py")
    spec = importlib.util.spec_from_file_location("trace_joint2_branch", module_path)
    module = importlib.util.module_from_spec(spec)
    assert spec is not None and spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def row_at(rows, target_deg: float):
    """按采样角查找输出行。"""
    for row in rows:
        if abs(float(row["s_deg"]) - target_deg) < 1e-9:
            return row
    raise KeyError(target_deg)


def test_real_roots_at_zero_include_origin_root():
    """s=0 时，四次方程的实根中应包含零位根 Y=0。"""
    module = load_module()

    roots = module.solve_real_y_roots(0.0)

    assert any(abs(root) < 1e-12 for root in roots)


def test_recover_xz_and_full_closure_for_zero_configuration():
    """零位根恢复出的 X、Z 和完整 MDH 闭环应回到初始位形。"""
    module = load_module()

    q = module.half_angle_input_from_s(0.0)
    x_value, z_value = module.recover_xz(0.0, q)
    angles = module.build_joint_angles_from_xyzs(x_value, 0.0, z_value, 0.0)
    residual = module.compute_closure_residual(angles)

    assert abs(x_value) < 1e-12
    assert abs(z_value) < 1e-12
    assert residual < 1e-12


def test_default_sampling_is_dense_near_270_and_skips_180():
    """默认采样应在 260°~280° 加密到 0.1°，并避开 180°。"""
    module = load_module()

    samples = module.default_sample_degrees()

    assert 180.0 not in samples
    assert 269.9 in samples
    assert 270.0 in samples
    assert 270.1 in samples
    assert 279.9 in samples


def test_trace_branch_smoke_and_initial_point(tmp_path: Path):
    """短程 branch trace 应能生成连续结果，且首点贴近零位。"""
    module = load_module()
    output_dir = tmp_path / "joint2_branch"
    sample_degrees = [0.0, 15.0, 30.0, 45.0, 60.0]

    result = module.trace_selected_branch(sample_degrees=sample_degrees, output_dir=output_dir)

    assert len(result["selected_rows"]) == len(sample_degrees)
    assert not result["failed_points"]
    first = result["selected_rows"][0]
    assert abs(first["delta_theta1_rad"]) < 1e-9
    assert abs(first["delta_theta3_rad"]) < 1e-9
    assert abs(first["delta_theta4_rad"]) < 1e-9
    assert first["closure_residual"] < 1e-12


def test_270_neighborhood_stays_continuous(tmp_path: Path):
    """270° 邻域的已选 branch 不应再出现大跳变。"""
    module = load_module()
    output_dir = tmp_path / "joint2_branch_dense"
    sample_degrees = [269.8, 269.9, 270.0, 270.1, 270.2]

    result = module.trace_selected_branch(sample_degrees=sample_degrees, output_dir=output_dir)
    rows = result["selected_rows"]

    prev_row = row_at(rows, 269.9)
    center_row = row_at(rows, 270.0)
    next_row = row_at(rows, 270.1)

    prev_diff = module.wrap_vector_norm([
        float(center_row[key]) - float(prev_row[key])
        for key in module.JOINT_KEYS
    ])
    next_diff = module.wrap_vector_norm([
        float(next_row[key]) - float(center_row[key])
        for key in module.JOINT_KEYS
    ])

    assert prev_diff < 0.2
    assert next_diff < 0.2
    assert float(center_row["closure_residual"]) < 1e-7


def test_export_outputs_exist(tmp_path: Path):
    """导出接口应生成 continuous 版本输出和 260°~280° 候选表。"""
    module = load_module()
    output_dir = tmp_path / "joint2_branch_outputs"
    sample_degrees = [0.0, 30.0, 60.0, 90.0, 120.0, 150.0, 210.0, 240.0, 270.0, 300.0, 330.0, 360.0]

    module.trace_selected_branch(sample_degrees=sample_degrees, output_dir=output_dir)

    expected = {
        "joint2_branch_candidates_260_280.csv",
        "joint2_branch_selected_continuous.csv",
        "joint2_branch_angles_continuous.png",
        "joint2_branch_residual_continuous.png",
        "joint2_branch_switch_report.txt",
    }
    actual = {path.name for path in output_dir.iterdir() if path.is_file()}
    assert expected.issubset(actual)
