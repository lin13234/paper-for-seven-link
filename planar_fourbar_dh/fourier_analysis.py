"""将四杆显式解拟合为线性趋势与傅里叶级数，并进行误差鉴定。"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from numpy.typing import ArrayLike, NDArray

from .fourbar_dh import forward_kinematics, solve_explicit


FloatArray = NDArray[np.float64]
JOINTS = ((2, "q4"), (4, "q6"), (6, "q1"))


plt.rcParams.update(
    {
        "font.family": "sans-serif",
        "font.sans-serif": ["Arial", "DejaVu Sans", "sans-serif"],
        "svg.fonttype": "none",
        "pdf.fonttype": 42,
        "font.size": 9,
        "axes.spines.right": False,
        "axes.spines.top": False,
        "legend.frameon": False,
    }
)


@dataclass(frozen=True)
class FourierFit:
    """单个关节的广义傅里叶拟合结果与误差指标。"""

    order: int
    coefficients: FloatArray
    predicted: FloatArray
    residual: FloatArray
    wrapped_residual: FloatArray
    rmse: float
    max_abs_error: float
    wrapped_rmse: float
    wrapped_max_abs_error: float
    r_squared: float


def build_fourier_matrix(x: ArrayLike, order: int = 5) -> FloatArray:
    """构造 [1, x, cos(x), sin(x), ..., cos(nx), sin(nx)] 设计矩阵。"""
    values = np.asarray(x, dtype=float)
    if values.ndim != 1:
        raise ValueError("x 必须是一维数组。")
    if order < 1:
        raise ValueError("order 必须至少为 1。")

    columns = [np.ones_like(values), values]
    for harmonic in range(1, order + 1):
        columns.append(np.cos(harmonic * values))
        columns.append(np.sin(harmonic * values))
    return np.column_stack(columns)


def wrap_to_pi(angle: ArrayLike) -> FloatArray:
    """把角度差包裹到 [-pi, pi)，用于比较物理关节姿态。"""
    values = np.asarray(angle, dtype=float)
    return (values + np.pi) % (2.0 * np.pi) - np.pi


def fit_fourier_with_trend(
    x: ArrayLike,
    y: ArrayLike,
    order: int = 5,
) -> FourierFit:
    """最小二乘拟合线性趋势加指定阶数傅里叶谐波。"""
    x_values = np.asarray(x, dtype=float)
    y_values = np.asarray(y, dtype=float)
    if x_values.shape != y_values.shape or x_values.ndim != 1:
        raise ValueError("x 与 y 必须是形状相同的一维数组。")

    matrix = build_fourier_matrix(x_values, order)
    coefficients, *_ = np.linalg.lstsq(matrix, y_values, rcond=None)
    predicted = matrix @ coefficients
    residual = predicted - y_values
    wrapped_residual = wrap_to_pi(residual)
    sum_squared = float(np.sum(residual**2))
    total_squared = float(np.sum((y_values - np.mean(y_values)) ** 2))
    r_squared = 1.0 - sum_squared / total_squared if total_squared > 0.0 else 1.0
    return FourierFit(
        order=order,
        coefficients=coefficients,
        predicted=predicted,
        residual=residual,
        wrapped_residual=wrapped_residual,
        rmse=float(np.sqrt(np.mean(residual**2))),
        max_abs_error=float(np.max(np.abs(residual))),
        wrapped_rmse=float(np.sqrt(np.mean(wrapped_residual**2))),
        wrapped_max_abs_error=float(np.max(np.abs(wrapped_residual))),
        r_squared=r_squared,
    )


def _dh_closure_errors(q_rows: FloatArray, length_scale: float) -> tuple[FloatArray, FloatArray]:
    """逐行计算拟合关节角代回完整七杆 DH 方程后的闭环残差。"""
    position = np.zeros(len(q_rows), dtype=float)
    rotation_error = np.zeros(len(q_rows), dtype=float)
    for row, q in enumerate(q_rows):
        end = forward_kinematics(q, length_scale)[-1]
        position[row] = np.linalg.norm(end[:3, 3])
        rotation = end[:3, :3]
        cosine = np.clip((np.trace(rotation) - 1.0) / 2.0, -1.0, 1.0)
        sine = 0.5 * np.linalg.norm(
            [
                rotation[2, 1] - rotation[1, 2],
                rotation[0, 2] - rotation[2, 0],
                rotation[1, 0] - rotation[0, 1],
            ]
        )
        rotation_error[row] = np.arctan2(sine, cosine)
    return position, rotation_error


def _coefficient_names(order: int) -> list[str]:
    """返回与设计矩阵列顺序一致的可读系数名。"""
    names = ["c0", "c1_linear"]
    for harmonic in range(1, order + 1):
        names.extend((f"a{harmonic}_cos", f"b{harmonic}_sin"))
    return names


def _write_coefficients(path: Path, fits: dict[str, FourierFit]) -> Path:
    """导出各关节傅里叶系数，所有系数单位均为弧度。"""
    names = _coefficient_names(next(iter(fits.values())).order)
    with path.open("w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.writer(handle)
        writer.writerow(["joint", "coefficient", "value_rad"])
        for joint, fit in fits.items():
            for name, value in zip(names, fit.coefficients):
                writer.writerow([joint, name, value])
    return path


def _write_metrics(
    path: Path,
    fits: dict[str, FourierFit],
    fit_position_error: FloatArray,
    fit_rotation_error: FloatArray,
) -> Path:
    """导出逐关节拟合误差和拟合角对应的完整 DH 闭环误差。"""
    with path.open("w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "joint",
                "rmse_deg",
                "max_abs_error_deg",
                "wrapped_rmse_deg",
                "wrapped_max_abs_error_deg",
                "r_squared",
                "max_dh_position_error",
                "max_dh_rotation_error_rad",
            ]
        )
        for joint, fit in fits.items():
            writer.writerow(
                [
                    joint,
                    np.degrees(fit.rmse),
                    np.degrees(fit.max_abs_error),
                    np.degrees(fit.wrapped_rmse),
                    np.degrees(fit.wrapped_max_abs_error),
                    fit.r_squared,
                    np.max(fit_position_error),
                    np.max(fit_rotation_error),
                ]
            )
    return path


def _write_comparison(
    path: Path,
    q2: FloatArray,
    exact_q: FloatArray,
    fits: dict[str, FourierFit],
) -> Path:
    """导出逐点显式解、傅里叶预测值及模 2pi 误差。"""
    with path.open("w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.writer(handle)
        header = ["q2_deg"]
        for _index, name in JOINTS:
            header.extend((f"{name}_explicit_deg", f"{name}_fourier_deg", f"{name}_wrapped_error_deg"))
        writer.writerow(header)
        for row in range(len(q2)):
            values = [np.degrees(q2[row])]
            for index, name in JOINTS:
                fit = fits[name]
                values.extend(
                    (
                        np.degrees(exact_q[row, index]),
                        np.degrees(fit.predicted[row]),
                        np.degrees(fit.wrapped_residual[row]),
                    )
                )
            writer.writerow(values)
    return path


def _plot_comparison(
    path: Path,
    q2: FloatArray,
    exact_q: FloatArray,
    fits: dict[str, FourierFit],
) -> Path:
    """绘制显式结果与傅里叶拟合的逐关节叠加图。"""
    colors = ("#4477AA", "#228833", "#CC6677")
    fig, axes = plt.subplots(3, 1, figsize=(7.2, 8.0), sharex=True, constrained_layout=True)
    x_deg = np.degrees(q2)
    for ax, (index, name), color in zip(axes, JOINTS, colors):
        ax.plot(x_deg, np.degrees(exact_q[:, index]), color=color, lw=2.0, label="Explicit")
        ax.plot(
            x_deg,
            np.degrees(fits[name].predicted),
            color="#222222",
            lw=1.2,
            ls="--",
            label="5th-order Fourier + trend",
        )
        ax.set_ylabel(fr"${name}$ (deg)")
        ax.grid(True, color="#DDDDDD", lw=0.6)
        ax.legend(ncols=2, loc="best")
    axes[0].set_title("Explicit solution versus generalized Fourier approximation")
    axes[-1].set_xlabel(r"Input joint angle $q_2$ (deg)")
    fig.savefig(path, dpi=300, bbox_inches="tight")
    plt.close(fig)
    return path


def _plot_errors(path: Path, q2: FloatArray, fits: dict[str, FourierFit]) -> Path:
    """绘制三个关节的模 2pi 角误差，便于直接判断物理姿态差异。"""
    colors = {"q4": "#4477AA", "q6": "#228833", "q1": "#CC6677"}
    fig, ax = plt.subplots(figsize=(7.2, 4.2))
    for name, fit in fits.items():
        ax.plot(
            np.degrees(q2),
            np.degrees(fit.wrapped_residual),
            lw=1.5,
            color=colors[name],
            label=fr"${name}$",
        )
    ax.axhline(0.0, color="#777777", lw=0.8, ls=":")
    ax.set_xlabel(r"Input joint angle $q_2$ (deg)")
    ax.set_ylabel(r"Wrapped error $q_{\mathrm{Fourier}}-q_{\mathrm{explicit}}$ (deg)")
    ax.set_title("Angular error modulo $2\\pi$")
    ax.grid(True, color="#DDDDDD", lw=0.6)
    ax.legend(ncols=3)
    fig.savefig(path, dpi=300, bbox_inches="tight")
    plt.close(fig)
    return path


def _write_report(
    path: Path,
    fits: dict[str, FourierFit],
    position_error: FloatArray,
    rotation_error: FloatArray,
) -> Path:
    """生成中文定量结论，区分同一函数与工程近似两个概念。"""
    max_wrapped_deg = max(np.degrees(fit.wrapped_max_abs_error) for fit in fits.values())
    max_position = float(np.max(position_error))
    max_rotation = float(np.max(rotation_error))
    strict_same = max_wrapped_deg < 1e-8 and max_position < 1e-10 and max_rotation < 1e-10
    engineering_close = max_wrapped_deg < 1.0 and max_position < 5e-2 and max_rotation < np.radians(1.0)

    if strict_same:
        conclusion = "在数值精度内可视为同一显式函数。"
    elif engineering_close:
        conclusion = "不是同一个解析表达式，但五阶傅里叶模型可作为较高精度工程近似。"
    else:
        conclusion = "五阶傅里叶模型与显式解不能视为同一个运动映射，需要提高阶数或使用分段模型。"

    lines = [
        "五阶广义傅里叶级数与显式解对比报告",
        "=" * 42,
        "拟合模型: c0 + c1*x + Σ[a_k cos(kx) + b_k sin(kx)], k=1..5",
        "",
    ]
    for joint, fit in fits.items():
        lines.extend(
            [
                f"{joint}:",
                f"  RMSE = {np.degrees(fit.rmse):.6e} deg",
                f"  最大绝对误差 = {np.degrees(fit.max_abs_error):.6e} deg",
                f"  模2π最大误差 = {np.degrees(fit.wrapped_max_abs_error):.6e} deg",
                f"  R² = {fit.r_squared:.12f}",
            ]
        )
    lines.extend(
        [
            "",
            f"拟合角代回 DH 后最大位置闭环误差 = {max_position:.6e}",
            f"拟合角代回 DH 后最大姿态闭环误差 = {max_rotation:.6e} rad",
            "",
            f"结论: {conclusion}",
            "说明: 傅里叶式是有限阶近似；只有误差达到数值零时，才能称为与显式解相同。",
        ]
    )
    path.write_text("\n".join(lines), encoding="utf-8")
    return path


def run_fourier_analysis(
    output_dir: str | Path = "planar_fourbar_output/fourier_comparison",
    steps: int = 721,
    order: int = 5,
    length_scale: float = 1.0,
) -> list[Path]:
    """计算显式解、拟合三个活动关节，并输出误差鉴定材料。"""
    if steps < 2:
        raise ValueError("steps 必须至少为 2。")
    target = Path(output_dir)
    target.mkdir(parents=True, exist_ok=True)
    q2 = np.linspace(0.0, 2.0 * np.pi, steps)
    explicit = solve_explicit(q2, l=length_scale)
    if not np.all(explicit.reachable):
        raise RuntimeError("当前输入区间包含不可达构型，不能直接执行整周拟合。")

    fits = {
        name: fit_fourier_with_trend(q2, explicit.q[:, index], order=order)
        for index, name in JOINTS
    }
    fitted_q = explicit.q.copy()
    for index, name in JOINTS:
        fitted_q[:, index] = fits[name].predicted
    fit_position_error, fit_rotation_error = _dh_closure_errors(fitted_q, length_scale)

    outputs = [
        _write_coefficients(target / "fourier_coefficients.csv", fits),
        _write_metrics(
            target / "fourier_error_metrics.csv",
            fits,
            fit_position_error,
            fit_rotation_error,
        ),
        _write_comparison(target / "explicit_vs_fourier.csv", q2, explicit.q, fits),
        _plot_comparison(target / "explicit_vs_fourier.png", q2, explicit.q, fits),
        _plot_errors(target / "fourier_error.png", q2, fits),
        _write_report(
            target / "fourier_analysis_report.txt",
            fits,
            fit_position_error,
            fit_rotation_error,
        ),
    ]

    for name, fit in fits.items():
        print(
            f"{name}: RMSE={np.degrees(fit.rmse):.6e} deg, "
            f"max={np.degrees(fit.max_abs_error):.6e} deg, "
            f"R^2={fit.r_squared:.12f}"
        )
    print(f"拟合角代回 DH 的最大位置误差: {np.max(fit_position_error):.6e}")
    print(f"拟合角代回 DH 的最大姿态误差: {np.max(fit_rotation_error):.6e} rad")
    return outputs


def main() -> None:
    """命令行入口。"""
    parser = argparse.ArgumentParser(description="对四杆显式解进行广义傅里叶拟合与误差鉴定。")
    parser.add_argument(
        "--output-dir",
        default="planar_fourbar_output/fourier_comparison",
    )
    parser.add_argument("--steps", type=int, default=721)
    parser.add_argument("--order", type=int, default=5)
    parser.add_argument("--length-scale", type=float, default=1.0)
    args = parser.parse_args()
    run_fourier_analysis(
        output_dir=args.output_dir,
        steps=args.steps,
        order=args.order,
        length_scale=args.length_scale,
    )


if __name__ == "__main__":
    main()
