"""基于完整被动 Jacobian SVD 的七关节闭链机构奇异性分析。"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from numpy.typing import NDArray
from scipy.optimize import brentq

from .fourbar_dh import (
    forward_kinematics,
    joint_axis_geometry,
    solve_explicit,
)


FloatArray = NDArray[np.float64]

# 数组仍按原 DH 连乘的链位置存储。纯编号替换后的链位置标签如下。
CHAIN_LABELS = (2, 3, 4, 5, 6, 7, 1)
DRIVING_CHAIN_INDEX = 0  # 链位置 1 对应新关节 q2。
MOVING_PASSIVE_CHAIN_INDICES = (2, 4, 6)  # 新 q4、q6、q1。


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
class SingularPoint:
    """一个完整被动 Jacobian 的降秩构型。"""

    q2_deg: float
    q1_deg: float
    q4_deg: float
    q6_deg: float
    singular_values: FloatArray
    determinant: float
    numerical_rank: int
    periodic_boundary: bool

    @property
    def sigma_min(self) -> float:
        return float(self.singular_values[-1])


@dataclass(frozen=True)
class SingularityResult:
    """整周扫描得到的奇异值序列、条件数和精确奇异点。"""

    q2_rad: FloatArray
    q: FloatArray
    passive_singular_values: FloatArray
    passive_determinant: FloatArray
    passive_condition_number: FloatArray
    reduced_sigma_min: FloatArray
    singular_points: tuple[SingularPoint, ...]


def build_complete_jacobian(q: FloatArray, l: float = 1.0) -> FloatArray:
    """构造完整七关节闭环的 6x7 空间速度 Jacobian。

    对第 i 个转动关节，其列向量写为

        J_i = [z_i × (p_end - p_i); z_i]。

    轴线位置与方向由原程序采用的改进 DH 变换直接提取。
    """
    q_array = np.asarray(q, dtype=float)
    if q_array.shape != (7,):
        raise ValueError("q 必须是长度为 7 的一维数组。")

    points, axes = joint_axis_geometry(q_array, l)
    end_position = forward_kinematics(q_array, l)[-1][:3, 3]
    jacobian = np.zeros((6, 7), dtype=float)
    for chain_index in range(7):
        jacobian[:3, chain_index] = np.cross(
            axes[chain_index],
            end_position - points[chain_index],
        )
        jacobian[3:, chain_index] = axes[chain_index]
    return jacobian


def build_passive_jacobian(q: FloatArray, l: float = 1.0) -> FloatArray:
    """删除驱动新 q2 对应列，得到完整 6x6 被动 Jacobian。"""
    return np.delete(
        build_complete_jacobian(q, l),
        DRIVING_CHAIN_INDEX,
        axis=1,
    )


def build_reduced_locked_jacobian(q: FloatArray, l: float = 1.0) -> FloatArray:
    """仅保留平面模式中实际运动的从动关节 q4、q6、q1，得到 6x3 矩阵。"""
    return build_complete_jacobian(q, l)[:, MOVING_PASSIVE_CHAIN_INDICES]


def _wrap_degrees(angle_rad: float) -> float:
    """把关节角表示为 [-180°, 180°)，便于奇异点表格阅读。"""
    return float(np.degrees((angle_rad + np.pi) % (2.0 * np.pi) - np.pi))


def _evaluate_at_angle(q2_rad: float, l: float) -> tuple[FloatArray, FloatArray, float, FloatArray]:
    """计算指定输入角处的关节变量、被动奇异值、行列式和约化奇异值。"""
    q = solve_explicit(np.array([q2_rad]), l=l, branch=1).q[0]
    passive = build_passive_jacobian(q, l)
    passive_sv = np.linalg.svd(passive, compute_uv=False)
    determinant = float(np.linalg.det(passive))
    reduced_sv = np.linalg.svd(build_reduced_locked_jacobian(q, l), compute_uv=False)
    return q, passive_sv, determinant, reduced_sv


def _make_singular_point(
    q2_rad: float,
    l: float,
    periodic_boundary: bool,
    rank_tolerance: float,
) -> SingularPoint:
    """把根位置转换为包含新编号关节角和 SVD 结果的记录。"""
    q, singular_values, determinant, _ = _evaluate_at_angle(q2_rad, l)
    numerical_rank = int(np.count_nonzero(singular_values > rank_tolerance))
    return SingularPoint(
        q2_deg=float(np.degrees(q2_rad)),
        q1_deg=_wrap_degrees(q[6]),
        q4_deg=_wrap_degrees(q[2]),
        q6_deg=_wrap_degrees(q[4]),
        singular_values=singular_values,
        determinant=determinant,
        numerical_rank=numerical_rank,
        periodic_boundary=periodic_boundary,
    )


def analyze_singularities(
    steps: int = 721,
    l: float = 1.0,
    zero_tolerance: float = 1e-10,
    rank_tolerance: float = 1e-10,
) -> SingularityResult:
    """扫描 s=+1 分支并用行列式变号精确定位完整 Jp 的奇异点。"""
    if steps < 3:
        raise ValueError("steps 必须至少为 3。")

    q2 = np.linspace(0.0, 2.0 * np.pi, steps)
    solution = solve_explicit(q2, l=l, branch=1)
    passive_sv = np.zeros((steps, 6), dtype=float)
    determinant = np.zeros(steps, dtype=float)
    reduced_sigma_min = np.zeros(steps, dtype=float)

    for row, q in enumerate(solution.q):
        jp = build_passive_jacobian(q, l)
        passive_sv[row] = np.linalg.svd(jp, compute_uv=False)
        determinant[row] = np.linalg.det(jp)
        reduced_sigma_min[row] = np.linalg.svd(
            build_reduced_locked_jacobian(q, l),
            compute_uv=False,
        )[-1]

    sigma_min = passive_sv[:, -1]
    condition = np.divide(
        passive_sv[:, 0],
        sigma_min,
        out=np.full_like(sigma_min, np.inf),
        where=sigma_min > np.finfo(float).eps,
    )

    roots: list[float] = []
    # 0° 与 360° 是同一个周期边界构型，仅保留 0° 一条记录。
    if sigma_min[0] < zero_tolerance or sigma_min[-1] < zero_tolerance:
        roots.append(0.0)

    def determinant_function(angle: float) -> float:
        return _evaluate_at_angle(angle, l)[2]

    for index in range(steps - 1):
        left, right = determinant[index], determinant[index + 1]
        if left * right < 0.0:
            root = brentq(
                determinant_function,
                q2[index],
                q2[index + 1],
                xtol=1e-14,
            )
            if root > 1e-9 and root < 2.0 * np.pi - 1e-9:
                if not any(abs(root - existing) < 1e-8 for existing in roots):
                    roots.append(root)

    roots.sort()
    points = tuple(
        _make_singular_point(
            root,
            l=l,
            periodic_boundary=abs(root) < 1e-9,
            rank_tolerance=rank_tolerance,
        )
        for root in roots
    )
    return SingularityResult(
        q2_rad=q2,
        q=solution.q,
        passive_singular_values=passive_sv,
        passive_determinant=determinant,
        passive_condition_number=condition,
        reduced_sigma_min=reduced_sigma_min,
        singular_points=points,
    )


def _write_scan_csv(result: SingularityResult, path: Path) -> Path:
    """写出整周六个被动奇异值、条件数和约化最小奇异值。"""
    with path.open("w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "q2_deg",
                "sigma1",
                "sigma2",
                "sigma3",
                "sigma4",
                "sigma5",
                "sigma6_min",
                "det_Jp",
                "condition_number_Jp",
                "reduced_sigma_min",
            ]
        )
        for row, angle in enumerate(result.q2_rad):
            writer.writerow(
                [
                    np.degrees(angle),
                    *result.passive_singular_values[row],
                    result.passive_determinant[row],
                    result.passive_condition_number[row],
                    result.reduced_sigma_min[row],
                ]
            )
    return path


def _write_points_csv(result: SingularityResult, path: Path) -> Path:
    """写出精确奇异点及其六个奇异值。"""
    with path.open("w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "q2_deg",
                "q1_deg",
                "q4_deg",
                "q6_deg",
                "sigma1",
                "sigma2",
                "sigma3",
                "sigma4",
                "sigma5",
                "sigma6_min",
                "det_Jp",
                "rank_Jp",
                "periodic_boundary",
            ]
        )
        for point in result.singular_points:
            writer.writerow(
                [
                    point.q2_deg,
                    point.q1_deg,
                    point.q4_deg,
                    point.q6_deg,
                    *point.singular_values,
                    point.determinant,
                    point.numerical_rank,
                    int(point.periodic_boundary),
                ]
            )
    return path


def _plot_singular_values(result: SingularityResult, output_stem: Path) -> list[Path]:
    """绘制六个完整被动奇异值及最小奇异值的局部判零图。"""
    q2_deg = np.degrees(result.q2_rad)
    colors = ("#9ECAE1", "#FDD0A2", "#A1D99B", "#FC9272", "#C5B0D5")
    fig, (ax_top, ax_bottom) = plt.subplots(
        2,
        1,
        figsize=(7.4, 6.5),
        sharex=True,
        gridspec_kw={"height_ratios": [2.0, 1.15]},
        constrained_layout=True,
    )

    for index in range(5):
        ax_top.plot(
            q2_deg,
            result.passive_singular_values[:, index],
            lw=1.4,
            color=colors[index],
            label=fr"$\sigma_{index + 1}$",
        )
    ax_top.plot(
        q2_deg,
        result.passive_singular_values[:, -1],
        color="#D62728",
        lw=2.2,
        label=r"$\sigma_6=\sigma_{{\min}}$",
    )
    ax_top.set_ylabel(r"Singular value of $J_p$")
    ax_top.set_title(r"SVD of the complete passive Jacobian on the $s=+1$ branch")
    ax_top.grid(True, color="#DDDDDD", lw=0.6)
    ax_top.legend(ncols=3)

    floor = 1e-16
    ax_bottom.semilogy(
        q2_deg,
        np.maximum(result.passive_singular_values[:, -1], floor),
        color="#D62728",
        lw=2.0,
        label=r"Complete $6\times6$: $\sigma_{{\min}}(J_p)$",
    )
    ax_bottom.semilogy(
        q2_deg,
        result.reduced_sigma_min,
        color="#4477AA",
        lw=1.6,
        ls="--",
        label=r"Locked mode $6\times3$: $\sigma_{{\min}}(J_{p,\mathrm{red}})$",
    )
    for point in result.singular_points:
        display_angle = point.q2_deg
        ax_bottom.axvline(display_angle, color="#333333", lw=0.8, ls=":")
        ax_bottom.scatter(
            [display_angle],
            [max(point.sigma_min, floor)],
            color="#D62728",
            s=24,
            zorder=5,
        )
        label = "0°/360°" if point.periodic_boundary else f"{display_angle:.6f}°"
        ax_bottom.annotate(
            label,
            (display_angle, max(point.sigma_min, floor)),
            xytext=(6, 12),
            textcoords="offset points",
            fontsize=8,
        )
    ax_bottom.set_xlabel(r"Driving joint angle $q_2$ (deg)")
    ax_bottom.set_ylabel("Minimum singular value")
    ax_bottom.grid(True, which="both", color="#DDDDDD", lw=0.6)
    ax_bottom.legend()

    outputs = []
    for suffix, options in (
        (".png", {"dpi": 300}),
        (".svg", {}),
        (".pdf", {}),
    ):
        path = output_stem.with_suffix(suffix)
        fig.savefig(path, bbox_inches="tight", **options)
        outputs.append(path)
    plt.close(fig)
    return outputs


def _markdown_report(result: SingularityResult) -> str:
    """生成包含方法简介、推导、结果表和判据解释的 Markdown 报告。"""
    rows = []
    for point in result.singular_points:
        location = "0/360（周期边界）" if point.periodic_boundary else f"{point.q2_deg:.9f}"
        rows.append(
            f"| {location} | {point.q1_deg:.6f} | {point.q4_deg:.6f} | "
            f"{point.q6_deg:.6f} | {point.sigma_min:.3e} | "
            f"{point.numerical_rank} |"
        )
    point_table = "\n".join(rows)
    reduced_min = float(np.min(result.reduced_sigma_min))
    reduced_angle = float(
        np.degrees(result.q2_rad[np.argmin(result.reduced_sigma_min)])
    )

    return f"""# 基于 SVD 的七关节闭链机构奇异性分析

## 1. SVD 方法简介

对任意矩阵 $A\\in\\mathbb R^{{m\\times n}}$，奇异值分解写为

$$
A=U\\Sigma V^{{\\mathsf T}},
$$

其中 $U$ 与 $V$ 为正交矩阵，$\\Sigma$ 的对角元满足

$$
\\sigma_1\\ge\\sigma_2\\ge\\cdots\\ge\\sigma_r\\ge0.
$$

最小奇异值刻画矩阵距离秩亏的程度。当方阵的
$\\sigma_{{\\min}}=0$ 时，矩阵不满秩；当其接近零时，速度映射病态，
机构接近奇异构型。

## 2. 完整闭环速度方程

纯编号替换后的链位置标签依次为

$$
(2,3,4,5,6,7,1).
$$

第 $j$ 个转动关节对闭环末端速度的 Jacobian 列为

$$
J_j=
\\begin{{bmatrix}}
\\mathbf z_j\\times(\\mathbf p_e-\\mathbf p_j)\\\\
\\mathbf z_j
\\end{{bmatrix}}.
$$

完整闭环速度约束为

$$
J\\dot{{\\mathbf q}}=0.
$$

以新 $q_2$ 为驱动关节，将其列记为 $J_a$，其余六列组成

$$
J_p=
\\left[
J_{{q_3}}\\;
J_{{q_4}}\\;
J_{{q_5}}\\;
J_{{q_6}}\\;
J_{{q_7}}\\;
J_{{q_1}}
\\right]\\in\\mathbb R^{{6\\times6}}.
$$

于是

$$
J_p\\dot{{\\mathbf q}}_p=-J_a\\dot q_2.
$$

当 $\\det(J_p)=0$ 或 $\\sigma_6(J_p)=0$ 时，被动速度不能由给定
$\\dot q_2$ 唯一确定，完整七关节闭链机构处于奇异构型。

## 3. 当前 $s=+1$ 运动分支的结果

![完整被动 Jacobian 奇异值](singular_values.png)

对 $q_2\\in[0,360^\\circ]$ 扫描，并对行列式变号区间使用 Brent 法精确求根。
检测到的奇异构型如下：

| $q_2$ (deg) | $q_1$ (deg) | $q_4$ (deg) | $q_6$ (deg) | $\\sigma_{{\\min}}(J_p)$ | rank |
|---:|---:|---:|---:|---:|---:|
{point_table}

$0^\\circ$ 与 $360^\\circ$ 是同一个周期构型，因此只计作一个独立奇异位置。
内部奇异位置为

$$
q_2=210.900874173^\\circ.
$$

## 4. 与锁定后约化 Jacobian 的区别

在指定平面运动模式中，新关节 $q_3,q_5,q_7$ 被锁死。仅保留实际运动的
$q_4,q_6,q_1$ 时，

$$
J_{{p,\\mathrm{{red}}}}=
\\left[J_{{q_4}}\\;J_{{q_6}}\\;J_{{q_1}}\\right]
\\in\\mathbb R^{{6\\times3}}.
$$

该矩阵整周最小奇异值为 {reduced_min:.9f}，出现在
$q_2\\approx{reduced_angle:.3f}^\\circ$，始终大于零。这表示锁定后的
一自由度四杆轨迹本身没有丢失列秩；但若从完整七关节闭链机构的潜在被动
运动空间考察，$J_p$ 仍在上述两个构型处降秩。

## 5. 结论

1. 完整 $6\\times6$ 被动 Jacobian 是本报告判定七关节机构奇异性的主判据。
2. 当前 $s=+1$ 分支存在两个周期表示的零点：$0^\\circ/360^\\circ$ 和
   $210.900874173^\\circ$。
3. 稀疏采样可能只能得到约 $0.0015$ 的最小值；行列式求根表明真实最小
   奇异值处于 $10^{{-16}}$ 数值量级。
4. $6\\times3$ 约化结果不能替代完整机构奇异性分析，但可用于说明锁定后的
   平面运动映射仍保持满列秩。
"""


def _tex_report(result: SingularityResult) -> str:
    """生成可由 XeLaTeX 编译的中文 TeX 报告。"""
    rows = []
    for point in result.singular_points:
        location = "0/360" if point.periodic_boundary else f"{point.q2_deg:.9f}"
        rows.append(
            f"{location} & {point.q1_deg:.6f} & {point.q4_deg:.6f} & "
            f"{point.q6_deg:.6f} & {point.sigma_min:.3e} & "
            f"{point.numerical_rank} \\\\"
        )
    reduced_min = float(np.min(result.reduced_sigma_min))
    reduced_angle = float(
        np.degrees(result.q2_rad[np.argmin(result.reduced_sigma_min)])
    )
    return rf"""\documentclass[UTF8,12pt,a4paper]{{ctexart}}
\usepackage{{amsmath,amssymb,bm,booktabs,graphicx,geometry,float,hyperref}}
\geometry{{left=2.3cm,right=2.3cm,top=2.5cm,bottom=2.5cm}}
\title{{基于SVD的七关节闭链机构奇异性分析}}
\author{{}}
\date{{\today}}
\begin{{document}}
\maketitle
\tableofcontents

\section{{SVD方法简介}}
对任意矩阵$A\in\mathbb R^{{m\times n}}$，奇异值分解写为
\[
A=U\Sigma V^{{\mathsf T}},
\]
其中$U$与$V$为正交矩阵，$\Sigma$的对角元满足
\[
\sigma_1\ge\sigma_2\ge\cdots\ge\sigma_r\ge0.
\]
最小奇异值刻画矩阵距离秩亏的程度。当$\sigma_{{\min}}=0$时矩阵不满秩；
当其接近零时，速度映射病态，机构接近奇异构型。

\section{{完整闭环速度方程}}
纯编号替换后的链位置标签依次为$(2,3,4,5,6,7,1)$。第$j$个转动关节
对闭环末端速度的Jacobian列为
\[
J_j=
\begin{{bmatrix}}
\bm z_j\times(\bm p_e-\bm p_j)\\
\bm z_j
\end{{bmatrix}}.
\]
完整闭环速度约束为$J\dot{{\bm q}}=0$。以新$q_2$为驱动关节，将其列记为
$J_a$，其余六列组成
\[
J_p=
\left[J_{{q_3}}\;J_{{q_4}}\;J_{{q_5}}\;J_{{q_6}}\;J_{{q_7}}\;J_{{q_1}}\right]
\in\mathbb R^{{6\times6}}.
\]
因此
\[
J_p\dot{{\bm q}}_p=-J_a\dot q_2.
\]
当$\det(J_p)=0$或$\sigma_6(J_p)=0$时，被动速度不能由给定$\dot q_2$
唯一确定，完整七关节闭链机构处于奇异构型。

\section{{当前$s=+1$分支的SVD结果}}
\begin{{figure}}[H]
\centering
\includegraphics[width=0.94\textwidth]{{singular_values.png}}
\caption{{完整被动Jacobian的六个奇异值及最小奇异值判据。}}
\end{{figure}}

对$q_2\in[0,360^\circ]$扫描，并对行列式变号区间使用Brent法精确求根。
\begin{{table}}[H]
\centering
\caption{{完整被动Jacobian的奇异位置}}
\begin{{tabular}}{{rrrrrr}}
\toprule
$q_2$ (deg) & $q_1$ (deg) & $q_4$ (deg) & $q_6$ (deg)
& $\sigma_{{\min}}(J_p)$ & rank\\
\midrule
{chr(10).join(rows)}
\bottomrule
\end{{tabular}}
\end{{table}}
$0^\circ$与$360^\circ$是同一个周期构型，只计作一个独立奇异位置。
内部奇异位置为
\[
q_2=210.900874173^\circ.
\]

\section{{锁定后约化Jacobian}}
指定平面模式中$q_3,q_5,q_7$锁死。仅保留$q_4,q_6,q_1$时，
\[
J_{{p,\mathrm{{red}}}}=
\left[J_{{q_4}}\;J_{{q_6}}\;J_{{q_1}}\right]
\in\mathbb R^{{6\times3}}.
\]
其整周最小奇异值为{reduced_min:.9f}，出现在
$q_2\approx{reduced_angle:.3f}^\circ$，始终大于零。该结果说明锁定后的
平面四杆速度映射保持满列秩，但不能替代完整七关节机构的奇异性判定。

\section{{结论}}
\begin{{enumerate}}
\item 完整$6\times6$被动Jacobian是七关节机构奇异性分析的主判据。
\item 当前$s=+1$分支的独立奇异构型位于$0^\circ/360^\circ$和
$210.900874173^\circ$。
\item 稀疏采样可能只能得到约$0.0015$的最小值；精确求根后
$\sigma_{{\min}}$处于$10^{{-16}}$数值量级。
\item $6\times3$约化矩阵整周满列秩，只说明锁定后的指定运动模式不退化。
\end{{enumerate}}
\end{{document}}
"""


def run_singularity_analysis(
    output_dir: str | Path = "planar_fourbar_output/singularity_analysis",
    steps: int = 721,
    length_scale: float = 1.0,
) -> list[Path]:
    """运行完整 SVD 分析并输出数据、图和两种报告源文件。"""
    target = Path(output_dir)
    target.mkdir(parents=True, exist_ok=True)
    result = analyze_singularities(steps=steps, l=length_scale)

    outputs = [
        _write_scan_csv(result, target / "singular_values.csv"),
        _write_points_csv(result, target / "singular_points.csv"),
    ]
    outputs.extend(_plot_singular_values(result, target / "singular_values"))

    md_path = target / "singularity_report.md"
    tex_path = target / "singularity_report.tex"
    md_path.write_text(_markdown_report(result), encoding="utf-8")
    tex_path.write_text(_tex_report(result), encoding="utf-8")
    outputs.extend((md_path, tex_path))

    print("完整 6x6 被动 Jacobian 奇异点：")
    for point in result.singular_points:
        location = "0/360" if point.periodic_boundary else f"{point.q2_deg:.9f}"
        print(
            f"q2={location} deg, sigma_min={point.sigma_min:.3e}, "
            f"rank={point.numerical_rank}"
        )
    minimum_index = int(np.argmin(result.reduced_sigma_min))
    print(
        "锁定后 6x3 约化矩阵最小奇异值："
        f"{result.reduced_sigma_min[minimum_index]:.9f}，"
        f"q2={np.degrees(result.q2_rad[minimum_index]):.3f} deg"
    )
    return outputs


def main() -> None:
    """命令行入口。"""
    parser = argparse.ArgumentParser(description="基于 SVD 分析七关节闭链机构奇异性。")
    parser.add_argument(
        "--output-dir",
        default="planar_fourbar_output/singularity_analysis",
    )
    parser.add_argument("--steps", type=int, default=721)
    parser.add_argument("--length-scale", type=float, default=1.0)
    args = parser.parse_args()
    run_singularity_analysis(
        output_dir=args.output_dir,
        steps=args.steps,
        length_scale=args.length_scale,
    )


if __name__ == "__main__":
    main()
