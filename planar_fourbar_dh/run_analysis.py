"""运行平面四杆显式解分析并导出 CSV 与图像。"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from .fourbar_dh import FourBarSolution, solve_explicit


plt.rcParams.update(
    {
        "font.family": "sans-serif",
        "font.sans-serif": ["Arial", "DejaVu Sans", "sans-serif"],
        "svg.fonttype": "none",
        "pdf.fonttype": 42,
        "font.size": 9,
        "axes.spines.right": False,
        "axes.spines.top": False,
        "axes.linewidth": 0.8,
        "legend.frameon": False,
    }
)


def write_csv(solution: FourBarSolution, path: Path) -> Path:
    """将输入角、活动关节角和闭环误差写入 CSV。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "q2_deg",
                "q4_deg",
                "q6_deg",
                "q1_deg",
                "reachable",
                "position_error",
                "rotation_error_rad",
            ]
        )
        angles_deg = np.degrees(solution.q)
        for index in range(len(solution.q)):
            writer.writerow(
                [
                    angles_deg[index, 0],
                    angles_deg[index, 2],
                    angles_deg[index, 4],
                    angles_deg[index, 6],
                    int(solution.reachable[index]),
                    solution.position_error[index],
                    solution.rotation_error[index],
                ]
            )
    return path


def _save_main_figure(fig: plt.Figure, output_dir: Path) -> list[Path]:
    """用同一 Matplotlib 图对象导出可编辑矢量图和高分辨率位图。"""
    outputs = []
    for suffix, options in (
        (".png", {"dpi": 300}),
        (".svg", {}),
        (".pdf", {}),
    ):
        path = output_dir / f"joint_angle_mapping{suffix}"
        fig.savefig(path, bbox_inches="tight", **options)
        outputs.append(path)
    return outputs


def plot_results(solution: FourBarSolution, output_dir: Path) -> list[Path]:
    """绘制关节角映射、DH 闭环误差和四个典型机构构型。"""
    output_dir.mkdir(parents=True, exist_ok=True)
    valid = solution.reachable
    q_deg = np.degrees(solution.q)
    q2_deg = q_deg[:, 0]
    palette = {"q4": "#4477AA", "q6": "#228833", "q1": "#CC6677"}
    outputs: list[Path] = []

    # 主证据图：三个活动从动关节关于输入关节 q2 的显式映射。
    fig, ax = plt.subplots(figsize=(7.2, 4.4))
    ax.plot(q2_deg[valid], q_deg[valid, 2], lw=2.0, color=palette["q4"], label=r"$q_4$")
    ax.plot(q2_deg[valid], q_deg[valid, 4], lw=2.0, color=palette["q6"], label=r"$q_6$")
    ax.plot(q2_deg[valid], q_deg[valid, 6], lw=2.0, color=palette["q1"], label=r"$q_1$")
    ax.axhline(0.0, color="#999999", lw=0.8, ls=":")
    ax.set_xlabel(r"Input joint angle $q_2$ (deg)")
    ax.set_ylabel("DH joint angle (deg)")
    ax.set_title("Explicit joint-angle mapping of the planar four-bar mode")
    ax.grid(True, color="#DDDDDD", lw=0.6)
    ax.legend(ncols=3)
    outputs.extend(_save_main_figure(fig, output_dir))
    plt.close(fig)

    # 验证图：完整七杆 DH 闭环的位置和姿态残差。
    fig, ax = plt.subplots(figsize=(7.2, 3.8))
    tiny = np.finfo(float).tiny
    ax.semilogy(
        q2_deg[valid],
        np.maximum(solution.position_error[valid], tiny),
        color="#4477AA",
        lw=1.5,
        label="Position closure error",
    )
    ax.semilogy(
        q2_deg[valid],
        np.maximum(solution.rotation_error[valid], tiny),
        color="#EE6677",
        lw=1.5,
        label="Rotation closure error (rad)",
    )
    ax.set_xlabel(r"Input joint angle $q_2$ (deg)")
    ax.set_ylabel("Closure error")
    ax.set_title("Residual of the complete seven-joint DH closure")
    ax.grid(True, which="both", color="#DDDDDD", lw=0.6)
    ax.legend()
    error_path = output_dir / "closure_error.png"
    fig.savefig(error_path, dpi=300, bbox_inches="tight")
    outputs.append(error_path)
    plt.close(fig)

    # 机构构型图：按样本索引选取四个代表位置，避免依赖步数整除关系。
    valid_indices = np.flatnonzero(valid)
    fractions = np.array([0.0, 0.25, 0.5, 0.75])
    selected = valid_indices[
        np.minimum(
            np.round(fractions * (len(valid_indices) - 1)).astype(int),
            len(valid_indices) - 1,
        )
    ]
    fig, axes = plt.subplots(2, 2, figsize=(7.0, 6.4), constrained_layout=True)
    for ax, index in zip(axes.flat, selected):
        polygon = np.vstack((solution.points[index], solution.points[index, 0]))
        ax.plot(polygon[:, 0], polygon[:, 1], "-o", color="#4477AA", lw=2.0, ms=5)
        for label, point in zip(("J2", "J4", "J6", "J1"), solution.points[index]):
            ax.annotate(label, point, xytext=(4, 5), textcoords="offset points")
        ax.set_title(fr"$q_2={q2_deg[index]:.1f}^\circ$")
        ax.set_aspect("equal", adjustable="box")
        ax.grid(True, color="#E5E5E5", lw=0.6)
        ax.set_xlabel("x / l")
        ax.set_ylabel("y / l")
    config_path = output_dir / "fourbar_configurations.png"
    fig.savefig(config_path, dpi=300, bbox_inches="tight")
    outputs.append(config_path)
    plt.close(fig)
    return outputs


def run_analysis(
    output_dir: str | Path = "planar_fourbar_output",
    steps: int = 721,
    q2_start_deg: float = 0.0,
    q2_end_deg: float = 360.0,
    length_scale: float = 1.0,
) -> list[Path]:
    """执行完整分析并返回所有已生成文件路径。"""
    if steps < 2:
        raise ValueError("steps 必须至少为 2。")
    target = Path(output_dir)
    q2 = np.radians(np.linspace(q2_start_deg, q2_end_deg, steps))
    solution = solve_explicit(q2, l=length_scale)
    outputs = [write_csv(solution, target / "joint_angles.csv")]
    outputs.extend(plot_results(solution, target))

    valid_count = int(np.count_nonzero(solution.reachable))
    print(f"有效输入角样本数: {valid_count}/{steps}")
    print(f"装配支路符号: {solution.branch_sign:+d}")
    if valid_count:
        print(f"最大位置闭环误差: {np.nanmax(solution.position_error):.3e}")
        print(f"最大姿态闭环误差: {np.nanmax(solution.rotation_error):.3e} rad")
    return outputs


def main() -> None:
    """命令行入口。"""
    parser = argparse.ArgumentParser(description="计算 2-4-6-1 平面四杆模式的显式解。")
    parser.add_argument("--output-dir", default="planar_fourbar_output")
    parser.add_argument("--steps", type=int, default=721)
    parser.add_argument("--q2-start-deg", type=float, default=0.0)
    parser.add_argument("--q2-end-deg", type=float, default=360.0)
    parser.add_argument("--length-scale", type=float, default=1.0)
    args = parser.parse_args()
    run_analysis(
        output_dir=args.output_dir,
        steps=args.steps,
        q2_start_deg=args.q2_start_deg,
        q2_end_deg=args.q2_end_deg,
        length_scale=args.length_scale,
    )


if __name__ == "__main__":
    main()
