"""
绘制显式解与傅里叶拟合结果的单图对比。

图中约定：
1. 三个关节 q1/q4/q6 全部叠加到同一个坐标系中。
2. 显式解使用实线。
3. 傅里叶拟合结果使用同色虚线。
4. 横轴统一使用输入角 q2，单位为 deg。
"""

from __future__ import annotations

from pathlib import Path

import matplotlib as mpl
import matplotlib.pyplot as plt
import pandas as pd


# 统一论文图风格，保证导出的 SVG/PDF 文字可编辑。
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
    """同时导出 PNG、SVG、PDF。"""
    fig.savefig(output_stem.with_suffix(".png"), dpi=600, bbox_inches="tight")
    fig.savefig(output_stem.with_suffix(".svg"), bbox_inches="tight")
    fig.savefig(output_stem.with_suffix(".pdf"), bbox_inches="tight")


def main() -> None:
    base_dir = Path(__file__).resolve().parent
    explicit_csv = base_dir.parent / "explicit_solution" / "joint_angles.csv"
    comparison_csv = base_dir / "explicit_vs_fourier.csv"

    # joint_angles.csv 作为显式解原始来源，
    # explicit_vs_fourier.csv 作为显式解与傅里叶拟合对照来源。
    explicit_df = pd.read_csv(explicit_csv)
    comparison_df = pd.read_csv(comparison_csv)

    # 先按 q2 对齐，再检查显式解列一致性，避免画错数据。
    merged_df = comparison_df.merge(
        explicit_df[["q2_deg", "q4_deg", "q6_deg", "q1_deg"]],
        on="q2_deg",
        how="inner",
        validate="one_to_one",
    )

    column_pairs = [
        ("q4_deg", "q4_explicit_deg", "q4_fourier_deg"),
        ("q6_deg", "q6_explicit_deg", "q6_fourier_deg"),
        ("q1_deg", "q1_explicit_deg", "q1_fourier_deg"),
    ]

    for source_col, explicit_col, _ in column_pairs:
        max_abs_diff = (merged_df[source_col] - merged_df[explicit_col]).abs().max()
        if max_abs_diff > 1e-9:
            raise ValueError(
                f"显式解数据不一致：{source_col} 与 {explicit_col} 的最大差值为 {max_abs_diff}"
            )

    fig_width_inch = 183 / 25.4
    fig_height_inch = 95 / 25.4
    fig, axis = plt.subplots(figsize=(fig_width_inch, fig_height_inch), constrained_layout=True)

    x_values = merged_df["q2_deg"]

    # 每个关节一组颜色，用线型区分显式解和傅里叶拟合。
    color_map = {
        "q4": "#1f4e79",
        "q6": "#2a7f62",
        "q1": "#c23b22",
    }

    for joint_name, explicit_col, fourier_col in [
        ("q4", "q4_explicit_deg", "q4_fourier_deg"),
        ("q6", "q6_explicit_deg", "q6_fourier_deg"),
        ("q1", "q1_explicit_deg", "q1_fourier_deg"),
    ]:
        axis.plot(
            x_values,
            merged_df[explicit_col],
            color=color_map[joint_name],
            linewidth=1.8,
            linestyle="-",
            label=f"{joint_name} explicit",
        )
        axis.plot(
            x_values,
            merged_df[fourier_col],
            color=color_map[joint_name],
            linewidth=1.6,
            linestyle="--",
            label=f"{joint_name} Fourier fit",
        )

    axis.set_xlabel(r"Input angle $q_2$ (deg)")
    axis.set_ylabel("Joint angle (deg)")
    axis.grid(True, linestyle=":", linewidth=0.6, alpha=0.5)
    axis.legend(loc="best", ncol=2)
    fig.suptitle("Comparison of Explicit Solution and Fourier Fit", fontsize=9)

    output_stem = base_dir / "explicit_vs_fourier_comparison_single_panel"
    save_figure(fig, output_stem)
    plt.close(fig)

    print(f"Figure exported to: {output_stem}")


if __name__ == "__main__":
    main()
