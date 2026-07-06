"""
验证最终分子中各因子的物理意义。

严格限制：
1. 不做任何新的符号消元；
2. 不做 Resultant / Groebner / RootOf；
3. 不对 Phi(Y,q) 做新的解析求解；
4. 只使用现有 MDH 定义、已有恢复关系，以及点式数值残差检查。

本脚本的目标：
1. 逐因子检查是否能对应真实闭环运动；
2. 对每个因子尝试恢复 Y -> X -> Z -> 七个关节角；
3. 代回完整 MDH，计算 ||T-I||_F；
4. 给出“真实运动分支 / 假根 / 孤立位形 / 另一装配模式”等判断；
5. 对因子 Y^2*q-4Y^2-4q+1=0 生成曲线与残差文件。
"""

from __future__ import annotations

import csv
import math
import time
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import least_squares

from mdh_closure_check import build_total_transform, closure_norm


PROJECT_DIR = Path(__file__).resolve().parent
DEFAULT_OUTPUT_DIR = PROJECT_DIR / "factor_physical_validation"

# 这里用较严格的闭环判据判断“是否满足完整 MDH”
RESIDUAL_OK_THRESHOLD = 1e-8

# 求解 X / Z 时只做点式多初值最小残差，不做任何连续延拓。
ANGLE_GUESSES = (
    -math.pi,
    -math.pi / 2.0,
    -math.pi / 4.0,
    0.0,
    math.pi / 4.0,
    math.pi / 2.0,
    math.pi,
)
MAX_NFEV = 200


@dataclass
class FactorSummary:
    """保存单个因子的最终判定。"""

    factor: str
    valid_sample_count: int
    mdh_ok_count: int
    min_residual: float
    max_residual: float
    mean_residual: float
    true_motion_branch: str
    satisfies_full_mdh: str
    elimination_spurious: str
    isolated_configuration_only: str
    other_assembly_mode: str
    note: str


def log(message: str) -> None:
    """打印进度，避免长时间无输出。"""
    now = time.strftime("%H:%M:%S")
    print(f"[{now}] {message}", flush=True)


def q_from_s_deg(s_deg: float) -> float:
    """由输入角 s 的角度值计算半角变量 q = tan(s/2)。"""
    return math.tan(math.radians(s_deg) / 2.0)


def y_from_Y(Y: float) -> float:
    """半角变量 Y 恢复为关节角 y。"""
    return 2.0 * math.atan(Y)


def x_from_X(X: float) -> float:
    """半角变量 X 恢复为关节角 x。"""
    return 2.0 * math.atan(X)


def z_from_Z(Z: float) -> float:
    """半角变量 Z 恢复为关节角 z。"""
    return 2.0 * math.atan(Z)


def principal_deg(angle_rad: float) -> float:
    """把弧度角包装到 [-180, 180) 度，便于展示。"""
    angle_deg = math.degrees(angle_rad)
    return ((angle_deg + 180.0) % 360.0) - 180.0


def z_formula(Y: float, q: float) -> float | None:
    """使用已有恢复关系 Z(Y,q)。遇到奇异分母时返回 None。"""
    denominator = Y**2 * q**2 + Y**2 - q**2 - 1.0
    if abs(denominator) < 1e-12:
        return None
    return 2.0 * Y * (1.0 - q**2) / denominator


def phi_value(Y: float, q: float) -> float:
    """已有的 Phi(Y,q) 因子，只用于验证，不做新求解。"""
    return (
        5.0 * Y**4 * q**3
        - 29.0 * Y**4 * q**2
        + 45.0 * Y**4 * q
        - 20.0 * Y**4
        + 5.0 * Y**2 * q**4
        - 50.0 * Y**2 * q**3
        + 88.0 * Y**2 * q**2
        - 50.0 * Y**2 * q
        + 5.0 * Y**2
        - 20.0 * q**4
        + 45.0 * q**3
        - 29.0 * q**2
        + 5.0 * q
    )


def c_factor_value(Y: float, q: float) -> float:
    """因子 Y^2*q - 4Y^2 - 4q + 1 的数值值。"""
    return Y**2 * q - 4.0 * Y**2 - 4.0 * q + 1.0


def full_closure_residual_from_xyz(x_rad: float, y_rad: float, z_rad: float, s_rad: float) -> float:
    """使用完整 7R MDH 闭环矩阵计算 ||T-I||_F。"""
    total = build_total_transform(
        theta1=x_rad,
        theta2=y_rad,
        theta3=math.pi + s_rad,
        theta4=z_rad,
        theta5=-s_rad,
        theta6=math.pi - y_rad,
        theta7=x_rad,
        length_scale=1.0,
    )
    return closure_norm(total)


def residual_vector_xz(variables: np.ndarray, y_rad: float, s_rad: float) -> np.ndarray:
    """固定 y 和 s，只对 x,z 做完整 MDH 闭环残差最小化。"""
    x_rad, z_rad = variables
    total = build_total_transform(
        theta1=float(x_rad),
        theta2=float(y_rad),
        theta3=math.pi + s_rad,
        theta4=float(z_rad),
        theta5=-s_rad,
        theta6=math.pi - y_rad,
        theta7=float(x_rad),
        length_scale=1.0,
    )
    return (total - np.eye(4, dtype=float)).reshape(-1)


def solve_xz_for_given_yq(Y: float, q: float) -> dict[str, float | bool | None]:
    """
    固定 Y 和 q，用多初值最小化完整闭环残差。
    这里只做点式验证，不做连续延拓，不做新的符号推导。
    """
    y_rad = y_from_Y(Y)
    s_rad = 2.0 * math.atan(q)

    best: dict[str, float | bool | None] = {
        "success": False,
        "residual": float("inf"),
        "x_rad": None,
        "z_rad": None,
        "X": None,
        "Z": None,
    }

    for x0 in ANGLE_GUESSES:
        for z0 in ANGLE_GUESSES:
            result = least_squares(
                fun=residual_vector_xz,
                x0=np.array([x0, z0], dtype=float),
                args=(y_rad, s_rad),
                max_nfev=MAX_NFEV,
                xtol=1e-12,
                ftol=1e-12,
                gtol=1e-12,
            )
            x_rad = float(result.x[0])
            z_rad = float(result.x[1])
            residual = full_closure_residual_from_xyz(x_rad, y_rad, z_rad, s_rad)
            if residual < float(best["residual"]):
                best = {
                    "success": residual < RESIDUAL_OK_THRESHOLD,
                    "residual": residual,
                    "x_rad": x_rad,
                    "z_rad": z_rad,
                    "X": math.tan(x_rad / 2.0),
                    "Z": math.tan(z_rad / 2.0),
                }
    return best


def evaluate_q_zero(sample_count: int) -> tuple[FactorSummary, list[dict[str, float | str]]]:
    """验证因子 q=0 的物理意义。"""
    y_samples_deg = np.linspace(-170.0, 170.0, sample_count)
    rows: list[dict[str, float | str]] = []

    for y_deg in y_samples_deg:
        Y_value = math.tan(math.radians(y_deg) / 2.0)
        solution = solve_xz_for_given_yq(Y_value, 0.0)
        rows.append(
            {
                "parameter_deg": float(y_deg),
                "Y": Y_value,
                "q": 0.0,
                "x_deg": principal_deg(float(solution["x_rad"])) if solution["x_rad"] is not None else "",
                "y_deg": y_deg,
                "z_deg": principal_deg(float(solution["z_rad"])) if solution["z_rad"] is not None else "",
                "residual": float(solution["residual"]),
            }
        )

    residuals = [float(row["residual"]) for row in rows]
    ok_count = sum(res < RESIDUAL_OK_THRESHOLD for res in residuals)
    summary = FactorSummary(
        factor="q=0",
        valid_sample_count=len(rows),
        mdh_ok_count=ok_count,
        min_residual=min(residuals),
        max_residual=max(residuals),
        mean_residual=float(np.mean(residuals)),
        true_motion_branch="否",
        satisfies_full_mdh="仅在 s=0 截面内检查",
        elimination_spurious="否" if ok_count > 0 else "是",
        isolated_configuration_only="否" if ok_count > 1 else "是",
        other_assembly_mode="可能",
        note="该因子把输入固定为 q=0，只能对应 s=0 的位形簇，不可能是 0°~360° 全域运动分支。",
    )
    return summary, rows


def evaluate_Y_fixed(factor_label: str, Y_value: float, sample_count: int) -> tuple[FactorSummary, list[dict[str, float | str]]]:
    """验证 Y=1 或 Y=-1 这类固定 Y 因子。"""
    s_samples_deg = np.linspace(0.0, 360.0, sample_count)
    rows: list[dict[str, float | str]] = []

    for s_deg in s_samples_deg:
        q_value = q_from_s_deg(float(s_deg))
        solution = solve_xz_for_given_yq(Y_value, q_value)
        rows.append(
            {
                "s_deg": float(s_deg),
                "q": q_value,
                "Y": Y_value,
                "x_deg": principal_deg(float(solution["x_rad"])) if solution["x_rad"] is not None else "",
                "y_deg": principal_deg(y_from_Y(Y_value)),
                "z_deg": principal_deg(float(solution["z_rad"])) if solution["z_rad"] is not None else "",
                "residual": float(solution["residual"]),
            }
        )

    residuals = [float(row["residual"]) for row in rows]
    ok_count = sum(res < RESIDUAL_OK_THRESHOLD for res in residuals)
    summary = FactorSummary(
        factor=factor_label,
        valid_sample_count=len(rows),
        mdh_ok_count=ok_count,
        min_residual=min(residuals),
        max_residual=max(residuals),
        mean_residual=float(np.mean(residuals)),
        true_motion_branch="是" if ok_count == len(rows) else "否",
        satisfies_full_mdh="是" if ok_count == len(rows) else ("部分点" if ok_count > 0 else "否"),
        elimination_spurious="否" if ok_count > 0 else "是",
        isolated_configuration_only="否" if ok_count > 1 else "是",
        other_assembly_mode="可能" if ok_count > 0 else "否",
        note="固定 Y 的因子如果成立，也更像特定装配模式而不是一般主运动分支。",
    )
    return summary, rows


def evaluate_q_one(sample_count: int) -> tuple[FactorSummary, list[dict[str, float | str]]]:
    """验证因子 q=1 的物理意义。"""
    y_samples_deg = np.linspace(-170.0, 170.0, sample_count)
    rows: list[dict[str, float | str]] = []

    for y_deg in y_samples_deg:
        Y_value = math.tan(math.radians(y_deg) / 2.0)
        solution = solve_xz_for_given_yq(Y_value, 1.0)
        rows.append(
            {
                "parameter_deg": float(y_deg),
                "Y": Y_value,
                "q": 1.0,
                "x_deg": principal_deg(float(solution["x_rad"])) if solution["x_rad"] is not None else "",
                "y_deg": y_deg,
                "z_deg": principal_deg(float(solution["z_rad"])) if solution["z_rad"] is not None else "",
                "residual": float(solution["residual"]),
            }
        )

    residuals = [float(row["residual"]) for row in rows]
    ok_count = sum(res < RESIDUAL_OK_THRESHOLD for res in residuals)
    summary = FactorSummary(
        factor="q=1",
        valid_sample_count=len(rows),
        mdh_ok_count=ok_count,
        min_residual=min(residuals),
        max_residual=max(residuals),
        mean_residual=float(np.mean(residuals)),
        true_motion_branch="否",
        satisfies_full_mdh="仅在 s=90° 截面内检查",
        elimination_spurious="否" if ok_count > 0 else "是",
        isolated_configuration_only="否" if ok_count > 1 else "是",
        other_assembly_mode="可能",
        note="该因子把输入固定为 q=1，即 s=90°，不可能单独构成 0°~360° 全域运动分支。",
    )
    return summary, rows


def F1_Y_candidates(q: float) -> list[float]:
    """根据用户指定公式恢复因子 Y^2*q-4Y^2-4q+1=0 的 Y。"""
    denominator = q - 4.0
    if abs(denominator) < 1e-12:
        return []
    value = (4.0 * q - 1.0) / denominator
    if value < -1e-12:
        return []
    if abs(value) < 1e-12:
        return [0.0]
    root = math.sqrt(max(value, 0.0))
    return [root, -root]


def evaluate_F1_branch(output_dir: Path, sample_count: int) -> FactorSummary:
    """
    验证因子 Y^2*q-4Y^2-4q+1=0。
    这里只用用户指定的显式 Y(q)，然后对每个 q 点直接做完整 MDH 数值闭环检查。
    """
    s_samples_deg = np.linspace(0.0, 360.0, sample_count)
    rows: list[dict[str, float | str]] = []

    for s_deg in s_samples_deg:
        q_value = q_from_s_deg(float(s_deg))
        for branch_index, Y_value in enumerate(F1_Y_candidates(q_value), start=1):
            solution = solve_xz_for_given_yq(Y_value, q_value)
            x_rad = float(solution["x_rad"]) if solution["x_rad"] is not None else float("nan")
            z_rad = float(solution["z_rad"]) if solution["z_rad"] is not None else float("nan")
            y_rad = y_from_Y(Y_value)
            rows.append(
                {
                    "branch": f"Y_branch_{branch_index}",
                    "s_deg": float(s_deg),
                    "q": q_value,
                    "Y": Y_value,
                    "X": float(solution["X"]) if solution["X"] is not None else "",
                    "Z": float(solution["Z"]) if solution["Z"] is not None else "",
                    "x_deg": principal_deg(x_rad) if not math.isnan(x_rad) else "",
                    "y_deg": principal_deg(y_rad),
                    "z_deg": principal_deg(z_rad) if not math.isnan(z_rad) else "",
                    "theta1_deg": principal_deg(x_rad) if not math.isnan(x_rad) else "",
                    "theta2_deg": principal_deg(y_rad),
                    "theta3_deg": 180.0 + float(s_deg),
                    "theta4_deg": principal_deg(z_rad) if not math.isnan(z_rad) else "",
                    "theta5_deg": -float(s_deg),
                    "theta6_deg": 180.0 - principal_deg(y_rad),
                    "theta7_deg": principal_deg(x_rad) if not math.isnan(x_rad) else "",
                    "residual": float(solution["residual"]),
                }
            )

    csv_path = output_dir / "factor_F1_branch_curves.csv"
    with csv_path.open("w", encoding="utf-8", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0].keys()) if rows else ["branch", "s_deg", "residual"])
        writer.writeheader()
        writer.writerows(rows)

    if rows:
        fig, axes = plt.subplots(2, 1, figsize=(10, 10), sharex=True)
        for branch_name in sorted({str(row["branch"]) for row in rows}):
            branch_rows = [row for row in rows if row["branch"] == branch_name]
            axes[0].plot([row["s_deg"] for row in branch_rows], [row["x_deg"] for row in branch_rows], label=f"{branch_name}: Δθ1")
            axes[0].plot([row["s_deg"] for row in branch_rows], [row["y_deg"] for row in branch_rows], linestyle="--", label=f"{branch_name}: Δθ2")
            axes[0].plot([row["s_deg"] for row in branch_rows], [row["z_deg"] for row in branch_rows], linestyle=":", label=f"{branch_name}: Δθ4")

            axes[1].plot([row["s_deg"] for row in branch_rows], [row["theta1_deg"] for row in branch_rows], label=f"{branch_name}: θ1")
            axes[1].plot([row["s_deg"] for row in branch_rows], [row["theta2_deg"] for row in branch_rows], label=f"{branch_name}: θ2")
            axes[1].plot([row["s_deg"] for row in branch_rows], [row["theta3_deg"] for row in branch_rows], label=f"{branch_name}: θ3")
            axes[1].plot([row["s_deg"] for row in branch_rows], [row["theta4_deg"] for row in branch_rows], label=f"{branch_name}: θ4")
            axes[1].plot([row["s_deg"] for row in branch_rows], [row["theta5_deg"] for row in branch_rows], label=f"{branch_name}: θ5")
            axes[1].plot([row["s_deg"] for row in branch_rows], [row["theta6_deg"] for row in branch_rows], label=f"{branch_name}: θ6")
            axes[1].plot([row["s_deg"] for row in branch_rows], [row["theta7_deg"] for row in branch_rows], label=f"{branch_name}: θ7")

        axes[0].set_ylabel("Joint angle (deg)")
        axes[0].set_title("Factor Y^2*q-4Y^2-4q+1=0: Δθ1, Δθ2, Δθ4")
        axes[0].grid(True, alpha=0.3)
        axes[0].legend(fontsize=8, ncol=2)

        axes[1].set_xlabel("s = Δθ3 (deg)")
        axes[1].set_ylabel("Joint angle (deg)")
        axes[1].set_title("Factor Y^2*q-4Y^2-4q+1=0: full seven-joint curves")
        axes[1].grid(True, alpha=0.3)
        axes[1].legend(fontsize=8, ncol=3)

        fig.tight_layout()
        fig.savefig(output_dir / "factor_F1_branch_curves.png", dpi=200)
        plt.close(fig)

    residuals = [float(row["residual"]) for row in rows] if rows else [float("inf")]
    ok_count = sum(res < RESIDUAL_OK_THRESHOLD for res in residuals)
    summary = FactorSummary(
        factor="Y^2*q-4*Y^2-4*q+1=0",
        valid_sample_count=len(rows),
        mdh_ok_count=ok_count,
        min_residual=min(residuals),
        max_residual=max(residuals),
        mean_residual=float(np.mean(residuals)),
        true_motion_branch="是" if ok_count == len(rows) and len(rows) > 0 else "否",
        satisfies_full_mdh="是" if ok_count == len(rows) and len(rows) > 0 else ("部分点" if ok_count > 0 else "否"),
        elimination_spurious="否" if ok_count > 0 else "是",
        isolated_configuration_only="否" if ok_count > 1 else "是",
        other_assembly_mode="可能" if ok_count > 0 else "否",
        note="该因子按用户指定的 Y=±sqrt((4q-1)/(q-4)) 直接恢复，再用完整 MDH 做数值验证。",
    )
    return summary


def phi_Y_candidates(q: float) -> list[float]:
    """
    对已知 Phi(Y,q)=0 只做数值根恢复。
    利用它关于 U=Y^2 是二次多项式这一事实，恢复实数 Y 候选。
    这不是新的消元，只是对现有因子做数值检查。
    """
    a = 5.0 * q**3 - 29.0 * q**2 + 45.0 * q - 20.0
    b = 5.0 * q**4 - 50.0 * q**3 + 88.0 * q**2 - 50.0 * q + 5.0
    c = -20.0 * q**4 + 45.0 * q**3 - 29.0 * q**2 + 5.0 * q

    if abs(a) < 1e-12:
        if abs(b) < 1e-12:
            return []
        u_roots = [-c / b]
    else:
        disc = b * b - 4.0 * a * c
        if disc < -1e-10:
            return []
        disc = max(disc, 0.0)
        sqrt_disc = math.sqrt(disc)
        u_roots = [(-b + sqrt_disc) / (2.0 * a), (-b - sqrt_disc) / (2.0 * a)]

    candidates: list[float] = []
    for u_value in u_roots:
        if u_value < -1e-10:
            continue
        u_value = max(u_value, 0.0)
        root = math.sqrt(u_value)
        if root < 1e-12:
            candidates.append(0.0)
        else:
            candidates.extend([root, -root])
    # 去重，避免同一根重复。
    unique_values: list[float] = []
    for value in candidates:
        if not any(abs(value - existing) < 1e-10 for existing in unique_values):
            unique_values.append(value)
    return unique_values


def evaluate_phi_branch(sample_count: int) -> tuple[FactorSummary, list[dict[str, float | str]]]:
    """验证 Phi(Y,q)=0 的物理意义。"""
    s_samples_deg = np.linspace(0.0, 360.0, sample_count)
    rows: list[dict[str, float | str]] = []

    for s_deg in s_samples_deg:
        q_value = q_from_s_deg(float(s_deg))
        for candidate_index, Y_value in enumerate(phi_Y_candidates(q_value), start=1):
            solution = solve_xz_for_given_yq(Y_value, q_value)
            rows.append(
                {
                    "candidate": f"Y_candidate_{candidate_index}",
                    "s_deg": float(s_deg),
                    "q": q_value,
                    "Y": Y_value,
                    "phi_check": phi_value(Y_value, q_value),
                    "x_deg": principal_deg(float(solution["x_rad"])) if solution["x_rad"] is not None else "",
                    "y_deg": principal_deg(y_from_Y(Y_value)),
                    "z_deg": principal_deg(float(solution["z_rad"])) if solution["z_rad"] is not None else "",
                    "residual": float(solution["residual"]),
                }
            )

    residuals = [float(row["residual"]) for row in rows] if rows else [float("inf")]
    ok_count = sum(res < RESIDUAL_OK_THRESHOLD for res in residuals)
    summary = FactorSummary(
        factor="Phi(Y,q)=0",
        valid_sample_count=len(rows),
        mdh_ok_count=ok_count,
        min_residual=min(residuals),
        max_residual=max(residuals),
        mean_residual=float(np.mean(residuals)),
        true_motion_branch="是" if ok_count > 0 and ok_count == len(rows) else "否",
        satisfies_full_mdh="是" if ok_count == len(rows) and len(rows) > 0 else ("部分点" if ok_count > 0 else "否"),
        elimination_spurious="否" if ok_count > 0 else "是",
        isolated_configuration_only="否" if ok_count > 1 else "是",
        other_assembly_mode="可能" if ok_count > 0 else "否",
        note="Phi 因子只做数值根恢复与完整 MDH 残差验证，不继续任何新的符号求解。",
    )
    return summary, rows


def write_summary_csv(output_dir: Path, summaries: list[FactorSummary]) -> None:
    """写出最终总表。"""
    path = output_dir / "factor_validation_summary.csv"
    with path.open("w", encoding="utf-8", newline="") as file:
        writer = csv.DictWriter(
            file,
            fieldnames=[
                "因子",
                "有效样本数",
                "满足完整MDH样本数",
                "最小残差",
                "最大残差",
                "平均残差",
                "是否真实运动分支",
                "是否满足完整MDH",
                "是否仅为消元假根",
                "是否仅对应孤立位形",
                "是否对应另一装配模式",
                "备注",
            ],
        )
        writer.writeheader()
        for item in summaries:
            writer.writerow(
                {
                    "因子": item.factor,
                    "有效样本数": item.valid_sample_count,
                    "满足完整MDH样本数": item.mdh_ok_count,
                    "最小残差": item.min_residual,
                    "最大残差": item.max_residual,
                    "平均残差": item.mean_residual,
                    "是否真实运动分支": item.true_motion_branch,
                    "是否满足完整MDH": item.satisfies_full_mdh,
                    "是否仅为消元假根": item.elimination_spurious,
                    "是否仅对应孤立位形": item.isolated_configuration_only,
                    "是否对应另一装配模式": item.other_assembly_mode,
                    "备注": item.note,
                }
            )


def write_report_md(output_dir: Path, summaries: list[FactorSummary]) -> None:
    """写出 Markdown 报告，便于直接阅读。"""
    lines = [
        "# 因子物理意义验证",
        "",
        "本报告只验证已有因子的物理意义，不继续任何新的消元、RootOf、Resultant 或 Gröbner。",
        "",
        "| 因子 | 是否真实运动分支 | 是否满足完整MDH | 是否仅为消元假根 | 是否仅对应孤立位形 | 是否对应另一装配模式 | 备注 |",
        "| --- | --- | --- | --- | --- | --- | --- |",
    ]
    for item in summaries:
        lines.append(
            f"| {item.factor} | {item.true_motion_branch} | {item.satisfies_full_mdh} | "
            f"{item.elimination_spurious} | {item.isolated_configuration_only} | {item.other_assembly_mode} | {item.note} |"
        )
    (output_dir / "factor_validation_report.md").write_text("\n".join(lines), encoding="utf-8")


def run_factor_validation(output_dir: Path = DEFAULT_OUTPUT_DIR, sample_count: int = 181) -> Path:
    """运行全部 6 个因子的物理意义验证。"""
    output_dir.mkdir(parents=True, exist_ok=True)
    log(f"Output directory: {output_dir}")
    log(f"Sample count per sweep: {sample_count}")

    summaries: list[FactorSummary] = []

    log("Evaluating factor q=0")
    summary_q0, _rows_q0 = evaluate_q_zero(sample_count)
    summaries.append(summary_q0)

    log("Evaluating factor Y=1")
    summary_y1, _rows_y1 = evaluate_Y_fixed("Y=1", 1.0, sample_count)
    summaries.append(summary_y1)

    log("Evaluating factor Y=-1")
    summary_yn1, _rows_yn1 = evaluate_Y_fixed("Y=-1", -1.0, sample_count)
    summaries.append(summary_yn1)

    log("Evaluating factor q=1")
    summary_q1, _rows_q1 = evaluate_q_one(sample_count)
    summaries.append(summary_q1)

    log("Evaluating factor Y^2*q-4Y^2-4q+1=0")
    summaries.append(evaluate_F1_branch(output_dir, sample_count))

    log("Evaluating factor Phi(Y,q)=0")
    summary_phi, _rows_phi = evaluate_phi_branch(sample_count)
    summaries.append(summary_phi)

    log("Writing summary table")
    write_summary_csv(output_dir, summaries)

    log("Writing markdown report")
    write_report_md(output_dir, summaries)

    return output_dir


def main() -> None:
    """命令行入口。"""
    run_factor_validation()
    log("Factor validation finished")


if __name__ == "__main__":
    main()
