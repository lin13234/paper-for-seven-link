"""
绘制显式解与傅里叶拟合结果的关节角对比图。

图中约定：
1. 实线表示显式解。
2. 虚线表示傅里叶拟合结果。
3. 横轴统一使用输入角 q2（单位：deg）。

输入文件：
- ../explicit_solution/joint_angles.csv
- ./explicit_vs_fourier.csv

输出文件：
- explicit_vs_fourier_comparison.png
- explicit_vs_fourier_comparison.svg
- explicit_vs_fourier_comparison.pdf
"""

from __future__ import annotations

from pathlib import Path

import matplotlib as mpl
import matplotlib.pyplot as plt
import pandas as pd


# 按论文图的常用风格设置全局参数，尽量保证导出的 SVG/PDF 文字可编辑。
mpl.rcParams.update(
    {
        "font.family": "sans-serif",
        "font.sans-serif": ["Arial", "Helvetica", "DejaVu Sans", "sans-serif"],
        "svg.fonttype": "none",
        "pdf.fonttype": 42,
        "font.size": 7,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "axes.linewidth": 0.8,
        "xtick.direction": "out",
        "ytick.direction": "out",
        "legend.frameon": False,
    }
)


def save_figure(fig: plt.Figure, output_stem: Path) -> None:
    """按照论文常用格式同时导出 PNG、SVG、PDF。"""
    fig.savefig(output_stem.with_suffix(".png"), dpi=600, bbox_inches="tight")
    fig.savefig(output_stem.with_suffix(".svg"), bbox_inches="tight")
    fig.savefig(output_stem.with_suffix(".pdf"), bbox_inches="tight")


def main() -> None:
    base_dir = Path(__file__).resolve().parent
    explicit_csv = base_dir.parent / "explicit_solution" / "joint_angles.csv"
    comparison_csv = base_dir / "explicit_vs_fourier.csv"

    # 读取两份数据。joint_angles.csv 用作显式解原始来源，
    # explicit_vs_fourier.csv 用作显式解/傅里叶拟合对照来源。
    explicit_df = pd.read_csv(explicit_csv)
    comparison_df = pd.read_csv(comparison_csv)

    # 用 joint_angles.csv 中的显式解列，与对照文件中的显式解列做一致性检查。
    merged_df = comparison_df.merge(
        explicit_df[["q2_deg", "q4_deg", "q6_deg", "q1_deg"]],
        on="q2_deg",
        how="inner",
        validate="one_to_one",
    )

    column_pairs = [
        ("q4_deg", "q4_explicit_deg", "q4_fourier_deg", r"$q_4$ (deg)"),
        ("q6_deg", "q6_explicit_deg", "q6_fourier_deg", r"$q_6$ (deg)"),
        ("q1_deg", "q1_explicit_deg", "q1_fourier_deg", r"$q_1$ (deg)"),
    ]

    for source_col, explicit_col, _, _ in column_pairs:
        max_abs_diff = (merged_df[source_col] - merged_df[explicit_col]).abs().max()
        if max_abs_diff > 1e-9:
            raise ValueError(
                f"显式解数据不一致：{source_col} 与 {explicit_col} 的最大差值为 {max_abs_diff}"
            )

    # 双栏宽度约 183 mm，这里换算成英寸用于 matplotlib。
    fig_width_inch = 183 / 25.4
    fig_height_inch = 125 / 25.4

    fig, axes = plt.subplots(
        nrows=3,
        ncols=1,
        figsize=(fig_width_inch, fig_height_inch),
        sharex=True,
        constrained_layout=True,
    )

    x_values = merged_df["q2_deg"]
    explicit_color = "#1f4e79"
    fourier_color = "#c23b22"

    # 这张图的核心结论是：
    # 在 q2 扫描过程中，傅里叶拟合曲线整体跟随显式解曲线，
    # 且通过虚线标注，可以直观看到两者的拟合接近程度。
    for axis, (_, explicit_col, fourier_col, y_label) in zip(axes, column_pairs):
        axis.plot(
            x_values,
            merged_df[explicit_col],
            color=explicit_color,
            linewidth=1.8,
            label="Explicit solution",
        )
        axis.plot(
            x_values,
            merged_df[fourier_col],
            color=fourier_color,
            linewidth=1.6,
            linestyle="--",
            label="Fourier fit",
        )
        axis.set_ylabel(y_label)
        axis.grid(True, linestyle=":", linewidth=0.6, alpha=0.5)
        axis.legend(loc="best")

    axes[-1].set_xlabel(r"Input angle $q_2$ (deg)")
    fig.suptitle("Comparison of Explicit Solution and Fourier Fit", fontsize=9)

    output_stem = base_dir / "explicit_vs_fourier_comparison"
    save_figure(fig, output_stem)
    plt.close(fig)

    print(f"Figure exported to: {output_stem}")


if __name__ == "__main__":
    main()
