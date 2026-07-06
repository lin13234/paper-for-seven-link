"""
只验证 Phi(Y,q)=0 拆出的四条显式 Y 分支。

严格限制：
1. 不继续任何新的消元；
2. 不继续任何 RootOf / Resultant / Groebner / solve；
3. 不再把 Phi 当整体判断；
4. 只对 Branch 1~4 分别做显式恢复与完整 MDH 残差验证。
"""

from __future__ import annotations

import csv
import math
import time
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from mdh_closure_check import build_total_transform, closure_norm


PROJECT_DIR = Path(__file__).resolve().parent
DEFAULT_OUTPUT_DIR = PROJECT_DIR / "phi_branch_decomposition"
RESIDUAL_OK_THRESHOLD = 1e-8
INITIAL_ANGLE_TOL_DEG = 1e-3


@dataclass
class BranchSummary:
    """保存单条分支的汇总结果。"""

    branch: str
    valid_count: int
    valid_intervals: str
    max_residual: float
    mean_residual: float
    passes_initial_pose: str
    initial_joint_angles: str
    continuity: str
    possible_main_branch: str
    possible_other_assembly_mode: str
    note: str


def log(message: str) -> None:
    """打印进度，避免长时间无输出。"""
    now = time.strftime("%H:%M:%S")
    print(f"[{now}] {message}", flush=True)


def q_from_s_deg(s_deg: float) -> float:
    """由输入角 s 的角度值计算 q = tan(s/2)。"""
    return math.tan(math.radians(s_deg) / 2.0)


def principal_deg(angle_rad: float) -> float:
    """把角度包装到 [-180, 180) 度。"""
    return ((math.degrees(angle_rad) + 180.0) % 360.0) - 180.0


def compute_ABC(q: float) -> tuple[float, float, float]:
    """计算 Phi(Y,q)=A(q)u^2+B(q)u+C(q) 的系数。"""
    a_value = 5.0 * q**3 - 29.0 * q**2 + 45.0 * q - 20.0
    b_value = 5.0 * q**4 - 50.0 * q**3 + 88.0 * q**2 - 50.0 * q + 5.0
    c_value = -20.0 * q**4 + 45.0 * q**3 - 29.0 * q**2 + 5.0 * q
    return a_value, b_value, c_value


def compute_u_values(q: float) -> tuple[float | None, float | None]:
    """按用户指定公式计算 u_plus 与 u_minus。"""
    a_value, b_value, c_value = compute_ABC(q)
    if not np.isfinite(a_value) or abs(a_value) < 1e-12:
        return None, None
    discriminant = b_value * b_value - 4.0 * a_value * c_value
    if discriminant < -1e-12:
        return None, None
    discriminant = max(discriminant, 0.0)
    sqrt_disc = math.sqrt(discriminant)
    denominator = 2.0 * a_value
    u_plus = (-b_value + sqrt_disc) / denominator
    u_minus = (-b_value - sqrt_disc) / denominator
    return u_plus, u_minus


def branch_Y(branch_index: int, q: float) -> float | None:
    """按 Branch 1~4 规则恢复 Y。"""
    u_plus, u_minus = compute_u_values(q)
    if branch_index in (1, 2):
        u_value = u_plus
    else:
        u_value = u_minus
    if u_value is None or not np.isfinite(u_value) or u_value < -1e-12:
        return None
    u_value = max(u_value, 0.0)
    root = math.sqrt(u_value)
    if branch_index in (1, 3):
        return root
    return -root


def recover_X(Y: float, q: float) -> float | None:
    """按用户指定公式恢复 X。"""
    denominator = (q - 1.0) * ((4.0 - q) * Y**2 + 4.0 * q - 1.0)
    if abs(denominator) < 1e-12 or not np.isfinite(denominator):
        return None
    value = -q * (Y**2 - 1.0) / denominator
    return value if np.isfinite(value) else None


def recover_Z(Y: float, q: float) -> float | None:
    """按用户指定公式恢复 Z。"""
    denominator = (q**2 + 1.0) * (Y**2 - 1.0)
    if abs(denominator) < 1e-12 or not np.isfinite(denominator):
        return None
    value = -2.0 * Y * (q**2 - 1.0) / denominator
    return value if np.isfinite(value) else None


def compute_full_mdh_residual(x_rad: float, y_rad: float, z_rad: float, s_rad: float) -> float:
    """代回完整 7R MDH 并计算 ||T-I||_F。"""
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


def sample_s_values(sample_count: int) -> np.ndarray:
    """对 s 从 0° 到 360° 采样，并显式避开 180°。"""
    values = np.linspace(0.0, 360.0, sample_count)
    return np.array([value for value in values if abs(value - 180.0) > 1e-9], dtype=float)


def contiguous_intervals(valid_s_values: list[float], step: float) -> list[tuple[float, float]]:
    """把有效样本点分组成连续区间。"""
    if not valid_s_values:
        return []
    sorted_values = sorted(valid_s_values)
    intervals: list[tuple[float, float]] = []
    start = sorted_values[0]
    previous = sorted_values[0]
    tolerance = step * 1.5 + 1e-9
    for value in sorted_values[1:]:
        if value - previous > tolerance:
            intervals.append((start, previous))
            start = value
        previous = value
    intervals.append((start, previous))
    return intervals


def analyze_continuity(valid_rows: list[dict[str, float | str]], step: float) -> str:
    """根据有效区间数量与角度跳变粗略判断连续性。"""
    if not valid_rows:
        return "否"
    intervals = contiguous_intervals([float(row["s_deg"]) for row in valid_rows], step)
    if len(intervals) != 1:
        return "否"

    x_unwrapped = np.unwrap(np.radians([float(row["delta_theta1_deg"]) for row in valid_rows]))
    y_unwrapped = np.unwrap(np.radians([float(row["delta_theta2_deg"]) for row in valid_rows]))
    z_unwrapped = np.unwrap(np.radians([float(row["delta_theta4_deg"]) for row in valid_rows]))
    max_jump_deg = max(
        np.max(np.abs(np.diff(np.degrees(x_unwrapped)))) if len(x_unwrapped) > 1 else 0.0,
        np.max(np.abs(np.diff(np.degrees(y_unwrapped)))) if len(y_unwrapped) > 1 else 0.0,
        np.max(np.abs(np.diff(np.degrees(z_unwrapped)))) if len(z_unwrapped) > 1 else 0.0,
    )
    return "是" if max_jump_deg < 45.0 else "否"


def initial_pose_status(valid_rows: list[dict[str, float | str]]) -> tuple[str, str]:
    """检查是否经过初始位形，并返回初始位形关节角。"""
    initial_candidates = [row for row in valid_rows if abs(float(row["s_deg"])) < 1e-9]
    if not initial_candidates:
        return "否", "无有效 s=0 样本"

    row = initial_candidates[0]
    x_deg = float(row["delta_theta1_deg"])
    y_deg = float(row["delta_theta2_deg"])
    z_deg = float(row["delta_theta4_deg"])
    passes = (
        abs(x_deg) < INITIAL_ANGLE_TOL_DEG
        and abs(y_deg) < INITIAL_ANGLE_TOL_DEG
        and abs(z_deg) < INITIAL_ANGLE_TOL_DEG
        and float(row["residual"]) < RESIDUAL_OK_THRESHOLD
    )
    angle_text = f"Δθ1={x_deg:.6f} deg, Δθ2={y_deg:.6f} deg, Δθ4={z_deg:.6f} deg"
    return ("是" if passes else "否"), angle_text


def classify_branch(valid_rows: list[dict[str, float | str]], all_rows: list[dict[str, float | str]], step: float) -> tuple[str, str, str, str]:
    """给出主分支和装配模式的粗分类。"""
    initial_passes, _initial_angles = initial_pose_status(valid_rows)
    continuity = analyze_continuity(valid_rows, step)
    valid_ratio = len(valid_rows) / len(all_rows) if all_rows else 0.0

    possible_main = "是" if initial_passes == "是" and continuity == "是" and valid_ratio > 0.7 else "否"
    possible_other_mode = "是" if initial_passes == "否" and len(valid_rows) > 0 else "否"
    return initial_passes, continuity, possible_main, possible_other_mode


def evaluate_branch(branch_index: int, s_values_deg: np.ndarray) -> tuple[BranchSummary, list[dict[str, float | str]]]:
    """对单条分支做显式恢复与完整 MDH 验证。"""
    branch_name = f"Branch {branch_index}"
    rows: list[dict[str, float | str]] = []
    step = float(s_values_deg[1] - s_values_deg[0]) if len(s_values_deg) > 1 else 1.0

    for s_deg in s_values_deg:
        q_value = q_from_s_deg(float(s_deg))
        Y_value = branch_Y(branch_index, q_value)
        valid_real = Y_value is not None and np.isfinite(Y_value)
        if valid_real:
            X_value = recover_X(float(Y_value), q_value)
            Z_value = recover_Z(float(Y_value), q_value)
        else:
            X_value = None
            Z_value = None

        if valid_real and X_value is not None and Z_value is not None:
            x_rad = 2.0 * math.atan(X_value)
            y_rad = 2.0 * math.atan(float(Y_value))
            z_rad = 2.0 * math.atan(Z_value)
            s_rad = math.radians(float(s_deg))
            residual = compute_full_mdh_residual(x_rad, y_rad, z_rad, s_rad)
            row = {
                "branch": branch_name,
                "s_deg": float(s_deg),
                "q": q_value,
                "Y": float(Y_value),
                "X": float(X_value),
                "Z": float(Z_value),
                "delta_theta1_deg": principal_deg(x_rad),
                "delta_theta2_deg": principal_deg(y_rad),
                "delta_theta3_deg": float(s_deg),
                "delta_theta4_deg": principal_deg(z_rad),
                "delta_theta5_deg": -float(s_deg),
                "delta_theta6_deg": -principal_deg(y_rad),
                "delta_theta7_deg": principal_deg(x_rad),
                "residual": residual,
                "valid_real": "是",
                "valid_mdh": "是" if residual < RESIDUAL_OK_THRESHOLD else "否",
            }
        else:
            row = {
                "branch": branch_name,
                "s_deg": float(s_deg),
                "q": q_value,
                "Y": "",
                "X": "",
                "Z": "",
                "delta_theta1_deg": "",
                "delta_theta2_deg": "",
                "delta_theta3_deg": float(s_deg),
                "delta_theta4_deg": "",
                "delta_theta5_deg": -float(s_deg),
                "delta_theta6_deg": "",
                "delta_theta7_deg": "",
                "residual": "",
                "valid_real": "否",
                "valid_mdh": "否",
            }
        rows.append(row)

    valid_rows = [row for row in rows if row["valid_mdh"] == "是"]
    real_rows = [row for row in rows if row["valid_real"] == "是" and row["residual"] != ""]
    if real_rows:
        residuals = [float(row["residual"]) for row in real_rows]
        intervals = contiguous_intervals([float(row["s_deg"]) for row in real_rows], step)
        interval_text = "; ".join(f"[{start:.1f}, {end:.1f}] deg" for start, end in intervals)
        initial_passes, initial_angles = initial_pose_status(valid_rows)
        continuity = analyze_continuity(valid_rows, step)
        possible_main, possible_other = "否", "否"
        if valid_rows:
            initial_passes, continuity, possible_main, possible_other = classify_branch(valid_rows, rows, step)
            _passes_again, initial_angles = initial_pose_status(valid_rows)
        summary = BranchSummary(
            branch=branch_name,
            valid_count=len(real_rows),
            valid_intervals=interval_text if interval_text else "无",
            max_residual=max(residuals),
            mean_residual=float(np.mean(residuals)),
            passes_initial_pose=initial_passes,
            initial_joint_angles=initial_angles,
            continuity=continuity,
            possible_main_branch=possible_main,
            possible_other_assembly_mode=possible_other,
            note="按显式 Y/X/Z 公式恢复后直接代回完整 MDH。",
        )
    else:
        summary = BranchSummary(
            branch=branch_name,
            valid_count=0,
            valid_intervals="无实数有效区间",
            max_residual=float("inf"),
            mean_residual=float("inf"),
            passes_initial_pose="否",
            initial_joint_angles="无有效样本",
            continuity="否",
            possible_main_branch="否",
            possible_other_assembly_mode="否",
            note="该分支在采样区间内没有得到实数恢复结果。",
        )
    return summary, rows


def save_branch_plot(output_dir: Path, branch_index: int, rows: list[dict[str, float | str]]) -> None:
    """保存单条分支的关节角图。"""
    real_rows = [row for row in rows if row["valid_real"] == "是" and row["residual"] != ""]
    fig, axes = plt.subplots(2, 1, figsize=(10, 10), sharex=True)

    if real_rows:
        s_values = [float(row["s_deg"]) for row in real_rows]
        axes[0].plot(s_values, [float(row["delta_theta1_deg"]) for row in real_rows], label="Δθ1")
        axes[0].plot(s_values, [float(row["delta_theta2_deg"]) for row in real_rows], label="Δθ2")
        axes[0].plot(s_values, [float(row["delta_theta4_deg"]) for row in real_rows], label="Δθ4")
        axes[0].set_title(f"Branch {branch_index}: Δθ1, Δθ2, Δθ4")
        axes[0].set_ylabel("Angle (deg)")
        axes[0].grid(True, alpha=0.3)
        axes[0].legend()

        axes[1].plot(s_values, [float(row["delta_theta1_deg"]) for row in real_rows], label="θ1=Δθ1")
        axes[1].plot(s_values, [float(row["delta_theta2_deg"]) for row in real_rows], label="θ2=Δθ2")
        axes[1].plot(s_values, [float(row["delta_theta3_deg"]) + 180.0 for row in real_rows], label="θ3=π+s")
        axes[1].plot(s_values, [float(row["delta_theta4_deg"]) for row in real_rows], label="θ4=Δθ4")
        axes[1].plot(s_values, [float(row["delta_theta5_deg"]) for row in real_rows], label="θ5=-s")
        axes[1].plot(s_values, [float(row["delta_theta6_deg"]) + 180.0 for row in real_rows], label="θ6=π-y")
        axes[1].plot(s_values, [float(row["delta_theta7_deg"]) for row in real_rows], label="θ7=Δθ1")
        axes[1].set_title(f"Branch {branch_index}: all seven joint curves")
        axes[1].set_xlabel("s = Δθ3 (deg)")
        axes[1].set_ylabel("Angle (deg)")
        axes[1].grid(True, alpha=0.3)
        axes[1].legend(fontsize=8, ncol=3)
    else:
        axes[0].text(0.5, 0.5, "No real valid samples", ha="center", va="center")
        axes[1].text(0.5, 0.5, "No real valid samples", ha="center", va="center")
        axes[0].set_axis_off()
        axes[1].set_axis_off()

    fig.tight_layout()
    fig.savefig(output_dir / f"branch{branch_index}_angles.png", dpi=200)
    plt.close(fig)


def save_residual_plot(output_dir: Path, branch_rows: list[list[dict[str, float | str]]]) -> None:
    """保存四条分支残差对比图。"""
    fig, ax = plt.subplots(figsize=(10, 6))
    for branch_index, rows in enumerate(branch_rows, start=1):
        real_rows = [row for row in rows if row["valid_real"] == "是" and row["residual"] != ""]
        if not real_rows:
            continue
        ax.plot(
            [float(row["s_deg"]) for row in real_rows],
            [float(row["residual"]) for row in real_rows],
            label=f"Branch {branch_index}",
        )
    ax.set_xlabel("s = Δθ3 (deg)")
    ax.set_ylabel("||T-I||_F")
    ax.set_title("Full MDH closure residuals for four Phi branches")
    ax.set_yscale("log")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(output_dir / "branch_residuals.png", dpi=200)
    plt.close(fig)


def save_summary_csv(output_dir: Path, summaries: list[BranchSummary]) -> None:
    """保存四条分支的汇总表。"""
    with (output_dir / "branch_decomposition.csv").open("w", encoding="utf-8", newline="") as file:
        writer = csv.DictWriter(
            file,
            fieldnames=[
                "Branch",
                "实数有效样本数",
                "实数有效区间",
                "最大闭环残差",
                "平均闭环残差",
                "是否经过初始位形",
                "初始位形关节角",
                "是否连续",
                "是否可能对应主运动分支",
                "是否可能对应其它装配模式",
                "备注",
            ],
        )
        writer.writeheader()
        for item in summaries:
            writer.writerow(
                {
                    "Branch": item.branch,
                    "实数有效样本数": item.valid_count,
                    "实数有效区间": item.valid_intervals,
                    "最大闭环残差": item.max_residual,
                    "平均闭环残差": item.mean_residual,
                    "是否经过初始位形": item.passes_initial_pose,
                    "初始位形关节角": item.initial_joint_angles,
                    "是否连续": item.continuity,
                    "是否可能对应主运动分支": item.possible_main_branch,
                    "是否可能对应其它装配模式": item.possible_other_assembly_mode,
                    "备注": item.note,
                }
            )


def save_report_txt(output_dir: Path, summaries: list[BranchSummary]) -> None:
    """保存文本报告。"""
    lines = [
        "Phi(Y,q)=0 four-branch decomposition report",
        "",
        "This report validates only Branch 1~4 built from u_plus/u_minus and the explicit Y/X/Z formulas.",
        "",
    ]
    for item in summaries:
        lines.append(item.branch)
        lines.append(f"  Real valid intervals: {item.valid_intervals}")
        lines.append(f"  Max residual: {item.max_residual}")
        lines.append(f"  Mean residual: {item.mean_residual}")
        lines.append(f"  Passes initial pose: {item.passes_initial_pose}")
        lines.append(f"  Initial joint angles: {item.initial_joint_angles}")
        lines.append(f"  Continuous: {item.continuity}")
        lines.append(f"  Possible main branch: {item.possible_main_branch}")
        lines.append(f"  Possible other assembly mode: {item.possible_other_assembly_mode}")
        lines.append(f"  Note: {item.note}")
        lines.append("")
    (output_dir / "branch_decomposition_report.txt").write_text("\n".join(lines), encoding="utf-8")


def run_phi_branch_decomposition(output_dir: Path = DEFAULT_OUTPUT_DIR, sample_count: int = 361) -> Path:
    """运行四条 Phi 显式分支的完整 MDH 验证。"""
    output_dir.mkdir(parents=True, exist_ok=True)
    log(f"Output directory: {output_dir}")
    s_values_deg = sample_s_values(sample_count)
    log(f"Using {len(s_values_deg)} sampled s values, avoiding 180 deg")

    summaries: list[BranchSummary] = []
    branch_rows: list[list[dict[str, float | str]]] = []

    for branch_index in range(1, 5):
        log(f"Evaluating Branch {branch_index}")
        summary, rows = evaluate_branch(branch_index, s_values_deg)
        summaries.append(summary)
        branch_rows.append(rows)
        save_branch_plot(output_dir, branch_index, rows)

    log("Writing branch residual comparison plot")
    save_residual_plot(output_dir, branch_rows)

    log("Writing branch summary CSV")
    save_summary_csv(output_dir, summaries)

    log("Writing branch report")
    save_report_txt(output_dir, summaries)

    return output_dir


def main() -> None:
    """命令行入口。"""
    run_phi_branch_decomposition()
    log("Phi branch decomposition finished")


if __name__ == "__main__":
    main()
