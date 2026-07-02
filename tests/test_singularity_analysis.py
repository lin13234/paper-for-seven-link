from pathlib import Path

import numpy as np

from planar_fourbar_dh.singularity_analysis import (
    analyze_singularities,
    build_complete_jacobian,
    run_singularity_analysis,
)
from planar_fourbar_dh.fourbar_dh import solve_explicit


def test_complete_passive_jacobian_is_singular_at_initial_configuration():
    """删除驱动新 q2 后，完整 6x6 被动 Jacobian 在初始构型应降秩。"""
    q = solve_explicit(np.array([0.0])).q[0]
    jacobian = build_complete_jacobian(q)
    singular_values = np.linalg.svd(np.delete(jacobian, 0, axis=1), compute_uv=False)
    assert singular_values[-1] < 1e-12


def test_internal_singular_root_is_located_near_210_900874_degrees():
    """整周内部奇异点应通过行列式变号而非离散最小值精确定位。"""
    result = analyze_singularities(steps=721)
    internal = [point for point in result.singular_points if not point.periodic_boundary]
    assert len(internal) == 1
    assert abs(internal[0].q2_deg - 210.90087417265173) < 1e-7
    assert internal[0].sigma_min < 1e-12


def test_reduced_locked_mode_remains_full_column_rank():
    """锁定 q3/q5/q7 后，运动中的 q4/q6/q1 约化 Jacobian 不应降列秩。"""
    result = analyze_singularities(steps=721)
    assert np.min(result.reduced_sigma_min) > 0.6


def test_singularity_analysis_writes_expected_outputs(tmp_path: Path):
    """分析入口应输出图、逐点数据、奇异点表和两种格式报告源文件。"""
    outputs = run_singularity_analysis(output_dir=tmp_path, steps=721)
    expected = {
        "singular_values.csv",
        "singular_points.csv",
        "singular_values.png",
        "singular_values.svg",
        "singular_values.pdf",
        "singularity_report.md",
        "singularity_report.tex",
    }
    assert expected.issubset({path.name for path in outputs})
    assert all(path.stat().st_size > 0 for path in outputs)
