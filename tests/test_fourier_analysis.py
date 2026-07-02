from pathlib import Path

import numpy as np

from planar_fourbar_dh.fourier_analysis import (
    build_fourier_matrix,
    fit_fourier_with_trend,
    run_fourier_analysis,
)


def test_fourier_with_trend_recovers_known_signal():
    """线性趋势与五阶谐波的最小二乘实现应恢复已知信号。"""
    x = np.linspace(0.0, 2.0 * np.pi, 721)
    y = 0.3 - 0.8 * x + 0.2 * np.cos(x) - 0.4 * np.sin(3.0 * x)
    fit = fit_fourier_with_trend(x, y, order=5)
    np.testing.assert_allclose(fit.predicted, y, atol=1e-12)
    assert fit.rmse < 1e-12
    assert fit.r_squared > 1.0 - 1e-12


def test_fourier_matrix_has_expected_columns():
    """设计矩阵列数应为常数、线性项以及每阶正余弦两列。"""
    matrix = build_fourier_matrix(np.array([0.0, 1.0]), order=5)
    assert matrix.shape == (2, 12)


def test_fourier_analysis_writes_comparison_outputs(tmp_path: Path):
    """完整分析必须输出拟合数据、系数、指标和对比图。"""
    outputs = run_fourier_analysis(output_dir=tmp_path, steps=361, order=5)
    expected = {
        "fourier_coefficients.csv",
        "fourier_error_metrics.csv",
        "explicit_vs_fourier.csv",
        "explicit_vs_fourier.png",
        "fourier_error.png",
        "fourier_analysis_report.txt",
    }
    assert expected.issubset({path.name for path in outputs})
    assert all(path.stat().st_size > 0 for path in outputs)
