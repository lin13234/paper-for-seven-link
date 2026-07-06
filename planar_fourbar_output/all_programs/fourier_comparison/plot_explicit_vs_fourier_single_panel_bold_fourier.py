"""
绘制显式解与傅里叶拟合结果的单图对比。

这个版本专门把傅里叶拟合虚线改成更深、更醒目的颜色，
方便在单图叠加时直接看清拟合曲线。
"""

from __future__ import annotations

from pathlib import Path

import matplotlib as mpl
import matplotlib.pyplot as plt
import pandas as pd


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

    explicit_df = pd.read_csv(explicit_csv)
    comparison_df = pd.read_csv(comparison_csv)

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

    # 显式解保留中等亮度，傅里叶虚线改成更深的高对比颜色。
    explicit_color_map = {
        "q4": "#5b8cc0",
        "q6": "#58a488",
        "q1": "#e07a5f",
    }
    fourier_color_map = {
        "q4": "#0b2545",
        "q6": "#0f5132",
        "q1": "#7f1d1d",
    }

    for joint_name, explicit_col, fourier_col in [
        ("q4", "q4_explicit_deg", "q4_fourier_deg"),
        ("q6", "q6_explicit_deg", "q6_fourier_deg"),
        ("q1", "q1_explicit_deg", "q1_fourier_deg"),
    ]:
        axis.plot(
            x_values,
            merged_df[explicit_col],
            color=explicit_color_map[joint_name],
            linewidth=1.8,
            linestyle="-",
            label=f"{joint_name} explicit",
        )
        axis.plot(
            x_values,
            merged_df[fourier_col],
            color=fourier_color_map[joint_name],
            linewidth=2.0,
            linestyle="--",
            label=f"{joint_name} Fourier fit",
        )

    axis.set_xlabel(r"Input angle $q_2$ (deg)")
    axis.set_ylabel("Joint angle (deg)")
    axis.grid(True, linestyle=":", linewidth=0.6, alpha=0.5)
    axis.legend(loc="best", ncol=2)
    fig.suptitle("Comparison of Explicit Solution and Fourier Fit", fontsize=9)

    output_stem = base_dir / "explicit_vs_fourier_comparison_single_panel_bold_fourier"
    save_figure(fig, output_stem)
    plt.close(fig)

    print(f"Figure exported to: {output_stem}")


if __name__ == "__main__":
    main()
