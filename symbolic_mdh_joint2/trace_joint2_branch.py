from __future__ import annotations

import csv
import math
from pathlib import Path
from typing import Iterable

import matplotlib
import numpy as np
from scipy.optimize import least_squares

matplotlib.use("Agg")
import matplotlib.pyplot as plt


OUTPUT_DIR = Path(__file__).resolve().parent
REFERENCE_CSV = Path(r"E:\hub\02论文正文\001paper\05_推导与附录\01_原始计算数据\01_可用核心数据\B_q2主分支与奇异性\j2_kinematics_table.csv")

LENGTH_SCALE = 1.0
CLOSURE_RESIDUAL_WEIGHT = 1.0
CONTINUITY_WEIGHT = 1e-2
PREDICTOR_WEIGHT = 1e-2
RESIDUAL_PRIORITY_THRESHOLD = 1e-7
SINGULARITY_DEGREES = 180.0
SPECIAL_Q_TOL = 1e-10
REAL_ROOT_TOL = 1e-9
UNIQUE_ROOT_TOL = 1e-7
SWITCH_CONTINUITY_THRESHOLD = 0.2

JOINT_KEYS = (
    "delta_theta1_rad",
    "delta_theta2_rad",
    "delta_theta3_rad",
    "delta_theta4_rad",
    "delta_theta5_rad",
    "delta_theta6_rad",
    "delta_theta7_rad",
)


def wrap_to_pi(angle: float) -> float:
    """把任意角度包装到 [-pi, pi) 区间。"""
    return math.atan2(math.sin(angle), math.cos(angle))


def wrap_vector_norm(values: Iterable[float]) -> float:
    """对一组角差做 wrap 后再取 2-范数。"""
    wrapped = np.array([wrap_to_pi(value) for value in values], dtype=float)
    return float(np.linalg.norm(wrapped))


def half_angle_input_from_s(s_rad: float) -> float:
    """q = tan(s/2)。"""
    return math.tan(0.5 * s_rad)


def psi_coefficients(q_value: float) -> list[float]:
    """四次方程 Psi(Y,q) 的系数，按 Y^4 到 Y^0 排列。"""
    q2 = q_value * q_value
    q4 = q2 * q2
    return [
        5.0 * q2 - 20.0,
        5.0 * q4 - 50.0 * q2 + 45.0,
        -29.0 * q4 + 88.0 * q2 - 29.0,
        45.0 * q4 - 50.0 * q2 + 5.0,
        -20.0 * q4 + 5.0 * q2,
    ]


def psi_value(y_value: float, q_value: float) -> float:
    """直接计算 Psi(Y,q)。"""
    coeffs = psi_coefficients(q_value)
    return float(np.polyval(np.array(coeffs, dtype=float), y_value))


def solve_real_y_roots(s_rad: float) -> list[float]:
    """求给定 s 下四次方程的全部实根并去重。"""
    q_value = half_angle_input_from_s(s_rad)
    roots = np.roots(np.array(psi_coefficients(q_value), dtype=float))

    real_roots: list[float] = []
    for root in roots:
        if abs(root.imag) > REAL_ROOT_TOL:
            continue
        candidate = float(root.real)
        if any(abs(candidate - existing) < UNIQUE_ROOT_TOL for existing in real_roots):
            continue
        real_roots.append(candidate)

    real_roots.sort()
    return real_roots


def recover_xz_candidates(y_value: float, q_value: float) -> list[tuple[float, float]]:
    """
    由解析式恢复 X,Z。
    对 q^2 = 1 的退化点，返回有限候选作为后续精修初值。
    """
    q2 = q_value * q_value
    z_denominator = (q2 - 1.0) * (y_value * y_value + 1.0)
    x_denominator = (y_value - 1.0) * (y_value * q2 - 4.0 * y_value - 4.0 * q2 + 1.0)

    if abs(z_denominator) > SPECIAL_Q_TOL and abs(x_denominator) > SPECIAL_Q_TOL:
        z_value = -2.0 * q_value * (y_value * y_value - 1.0) / z_denominator
        x_value = y_value * (q2 - 1.0) / x_denominator
        return [(x_value, z_value)]

    if abs(q2 - 1.0) > SPECIAL_Q_TOL:
        return []

    y_sign = 1.0 if y_value >= 0.0 else -1.0
    if y_sign > 0.0:
        x_candidates = [math.sqrt(5.0 / 3.0), -math.sqrt(5.0 / 3.0)]
    else:
        discriminant = math.sqrt(460.0)
        x_candidates = [(20.0 + discriminant) / 6.0, (20.0 - discriminant) / 6.0]

    z_scale = q_value * y_sign / 2.0
    return [(x_value, z_scale * x_value) for x_value in x_candidates]


def recover_xz(y_value: float, q_value: float) -> tuple[float, float]:
    """返回默认的 X,Z 恢复结果。"""
    candidates = recover_xz_candidates(y_value, q_value)
    if not candidates:
        raise ValueError("No valid X/Z candidate recovered.")
    return candidates[0]


def build_joint_angles_from_xyzs(x_value: float, y_value: float, z_value: float, s_rad: float) -> dict[str, float]:
    """根据 X,Y,Z,s 恢复 7 个关节增量角。"""
    delta_theta1 = 2.0 * math.atan(x_value)
    delta_theta2 = s_rad
    delta_theta3 = 2.0 * math.atan(y_value)
    delta_theta4 = 2.0 * math.atan(z_value)
    delta_theta5 = -delta_theta3
    delta_theta6 = -delta_theta2
    delta_theta7 = delta_theta1

    return {
        "delta_theta1_rad": delta_theta1,
        "delta_theta2_rad": delta_theta2,
        "delta_theta3_rad": delta_theta3,
        "delta_theta4_rad": delta_theta4,
        "delta_theta5_rad": delta_theta5,
        "delta_theta6_rad": delta_theta6,
        "delta_theta7_rad": delta_theta7,
    }


def mdh_matrix_numeric(alpha_prev: float, a_prev: float, theta_i: float, d_i: float) -> np.ndarray:
    """数值版 MDH 齐次矩阵。"""
    cos_alpha = math.cos(alpha_prev)
    sin_alpha = math.sin(alpha_prev)
    cos_theta = math.cos(theta_i)
    sin_theta = math.sin(theta_i)
    return np.array(
        [
            [cos_theta, -sin_theta, 0.0, a_prev],
            [sin_theta * cos_alpha, cos_theta * cos_alpha, -sin_alpha, -d_i * sin_alpha],
            [sin_theta * sin_alpha, cos_theta * sin_alpha, cos_alpha, d_i * cos_alpha],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


def build_total_transform_from_angles(angles: dict[str, float], length_scale: float = LENGTH_SCALE) -> np.ndarray:
    """把恢复出的关节角代回完整 MDH 闭环。"""
    l_value = float(length_scale)
    five_l_over_two = 2.5 * l_value
    half_pi = math.pi / 2.0

    theta1 = angles["delta_theta1_rad"]
    theta2 = angles["delta_theta2_rad"]
    theta3 = math.pi + angles["delta_theta3_rad"]
    theta4 = angles["delta_theta4_rad"]
    theta5 = angles["delta_theta5_rad"]
    theta6 = math.pi + angles["delta_theta6_rad"]
    theta7 = angles["delta_theta7_rad"]

    matrices = [
        mdh_matrix_numeric(0.0, l_value, theta1, 0.0),
        mdh_matrix_numeric(half_pi, 2.0 * l_value, theta2, five_l_over_two),
        mdh_matrix_numeric(half_pi, 0.0, theta3, 0.0),
        mdh_matrix_numeric(half_pi, five_l_over_two, theta4, 0.0),
        mdh_matrix_numeric(half_pi, five_l_over_two, theta5, 0.0),
        mdh_matrix_numeric(half_pi, 0.0, theta6, five_l_over_two),
        mdh_matrix_numeric(half_pi, 2.0 * l_value, theta7, 0.0),
    ]

    total = np.eye(4, dtype=float)
    for matrix in matrices:
        total = total @ matrix
    return total


def compute_closure_residual(angles: dict[str, float], length_scale: float = LENGTH_SCALE) -> float:
    """计算完整闭环的 Frobenius 残差。"""
    total = build_total_transform_from_angles(angles, length_scale=length_scale)
    return float(np.linalg.norm(total - np.eye(4, dtype=float), ord="fro"))


def closure_residual_vector_for_xz(xz_values: np.ndarray, y_value: float, s_rad: float) -> np.ndarray:
    """固定 Y,s 时，对 X,Z 做局部精修使用的残差向量。"""
    x_value = float(xz_values[0])
    z_value = float(xz_values[1])
    angles = build_joint_angles_from_xyzs(x_value, y_value, z_value, s_rad)
    total = build_total_transform_from_angles(angles, length_scale=LENGTH_SCALE)
    return (total - np.eye(4, dtype=float)).reshape(-1)


def build_candidate_from_xyz(
    s_deg: float,
    s_rad: float,
    q_value: float,
    y_value: float,
    x_value: float,
    z_value: float,
    root_rank: int,
    xz_rank: int,
) -> dict[str, float | int | str]:
    """统一构造单个候选记录。"""
    angles = build_joint_angles_from_xyzs(x_value, y_value, z_value, s_rad)
    residual = compute_closure_residual(angles)
    return {
        "s_deg": s_deg,
        "s_rad": s_rad,
        "q": q_value,
        "Y": y_value,
        "X": x_value,
        "Z": z_value,
        "root_rank": root_rank,
        "xz_rank": xz_rank,
        "root_key": f"{root_rank}:{xz_rank}",
        "branch_id": f"{root_rank}:{xz_rank}",
        "psi_residual": abs(psi_value(y_value, q_value)),
        "closure_residual": residual,
        **angles,
    }


def build_special_candidates_for_sample(s_deg: float, s_rad: float, q_value: float) -> list[dict[str, float | int | str]]:
    """
    q=±1 时，用邻域极限做初值，再在当前 s 上对 X,Z 做 least-squares 精修。
    """
    exact_y_roots = solve_real_y_roots(s_rad)
    epsilon_deg = 0.01
    seed_degrees = [s_deg - epsilon_deg, s_deg + epsilon_deg]
    seeds: list[dict[str, float | int | str]] = []
    for seed_deg in seed_degrees:
        seeds.extend(build_candidate_rows_for_sample(seed_deg))

    candidates: list[dict[str, float | int | str]] = []
    for root_rank, y_value in enumerate(exact_y_roots):
        matching_seeds = [seed for seed in seeds if abs(float(seed["Y"]) - y_value) < 0.2]
        for xz_rank, seed in enumerate(matching_seeds):
            initial = np.array([float(seed["X"]), float(seed["Z"])], dtype=float)
            result = least_squares(
                closure_residual_vector_for_xz,
                x0=initial,
                args=(y_value, s_rad),
                xtol=1e-14,
                ftol=1e-14,
                gtol=1e-14,
                max_nfev=200,
            )
            candidate = build_candidate_from_xyz(
                s_deg=s_deg,
                s_rad=s_rad,
                q_value=q_value,
                y_value=y_value,
                x_value=float(result.x[0]),
                z_value=float(result.x[1]),
                root_rank=root_rank,
                xz_rank=xz_rank,
            )
            duplicate = False
            for existing in candidates:
                if (
                    abs(float(existing["Y"]) - float(candidate["Y"])) < 1e-9
                    and abs(float(existing["X"]) - float(candidate["X"])) < 1e-6
                    and abs(float(existing["Z"]) - float(candidate["Z"])) < 1e-6
                ):
                    duplicate = True
                    break
            if not duplicate:
                candidates.append(candidate)
    return candidates


def build_candidate_rows_for_sample(s_deg: float) -> list[dict[str, float | int | str]]:
    """在单个输入角处生成全部候选。"""
    s_rad = math.radians(s_deg)
    q_value = half_angle_input_from_s(s_rad)
    if abs(q_value * q_value - 1.0) < SPECIAL_Q_TOL:
        return build_special_candidates_for_sample(s_deg, s_rad, q_value)

    y_roots = solve_real_y_roots(s_rad)
    candidates: list[dict[str, float | int | str]] = []
    for root_rank, y_value in enumerate(y_roots):
        xz_candidates = recover_xz_candidates(y_value, q_value)
        for xz_rank, (x_value, z_value) in enumerate(xz_candidates):
            candidates.append(
                build_candidate_from_xyz(
                    s_deg=s_deg,
                    s_rad=s_rad,
                    q_value=q_value,
                    y_value=y_value,
                    x_value=x_value,
                    z_value=z_value,
                    root_rank=root_rank,
                    xz_rank=xz_rank,
                )
            )
    return candidates


def continuity_error(current: dict[str, float | int | str], previous: dict[str, float | int | str]) -> float:
    """7 个关节角的 wrap_to_pi 差值范数。"""
    differences = [float(current[key]) - float(previous[key]) for key in JOINT_KEYS]
    return wrap_vector_norm(differences)


def predictor_error(
    current: dict[str, float | int | str],
    previous: dict[str, float | int | str],
    previous2: dict[str, float | int | str] | None,
) -> float:
    """前两点线性预测误差。"""
    if previous2 is None:
        return 0.0
    predicted = [
        float(previous[key]) + (float(previous[key]) - float(previous2[key]))
        for key in JOINT_KEYS
    ]
    differences = [float(current[key]) - predicted[index] for index, key in enumerate(JOINT_KEYS)]
    return wrap_vector_norm(differences)


def zero_pose_error(candidate: dict[str, float | int | str]) -> float:
    """首点专用评分：越接近零位越优先。"""
    values = [
        float(candidate["delta_theta1_rad"]),
        float(candidate["delta_theta3_rad"]),
        float(candidate["delta_theta4_rad"]),
    ]
    return float(np.linalg.norm(np.array(values, dtype=float)))


def choose_initial_candidate(candidates: list[dict[str, float | int | str]]) -> dict[str, float | int | str] | None:
    """首点选择最接近零位的候选。"""
    if not candidates:
        return None
    return min(candidates, key=lambda item: (zero_pose_error(item), float(item["closure_residual"])))


def score_candidate(
    candidate: dict[str, float | int | str],
    previous: dict[str, float | int | str],
    previous2: dict[str, float | int | str] | None,
) -> tuple[tuple[float, float, float], dict[str, float]]:
    """计算单个候选的连续跟踪评分。"""
    residual = float(candidate["closure_residual"])
    continuity = continuity_error(candidate, previous)
    predictor = predictor_error(candidate, previous, previous2)
    weighted = (
        CLOSURE_RESIDUAL_WEIGHT * residual
        + CONTINUITY_WEIGHT * continuity
        + PREDICTOR_WEIGHT * predictor
    )

    if residual < RESIDUAL_PRIORITY_THRESHOLD:
        key = (0.0, CONTINUITY_WEIGHT * continuity + PREDICTOR_WEIGHT * predictor, residual)
    else:
        key = (1.0, weighted, residual)

    metrics = {
        "continuity_error": continuity,
        "predictor_error": predictor,
        "selection_score": weighted,
        "score": weighted,
        "priority_group": key[0],
    }
    return key, metrics


def choose_continuous_candidate(
    candidates: list[dict[str, float | int | str]],
    previous: dict[str, float | int | str],
    previous2: dict[str, float | int | str] | None,
) -> tuple[dict[str, float | int | str] | None, list[dict[str, float | int | str]]]:
    """按强连续跟踪策略选根，并返回带评分的全部候选。"""
    if not candidates:
        return None, []

    scored_candidates: list[dict[str, float | int | str]] = []
    best_key: tuple[float, float, float] | None = None
    best_item: dict[str, float | int | str] | None = None

    for candidate in candidates:
        key, metrics = score_candidate(candidate, previous, previous2)
        enriched = dict(candidate)
        enriched.update(metrics)
        scored_candidates.append(enriched)
        if best_key is None or key < best_key:
            best_key = key
            best_item = enriched

    return best_item, scored_candidates


def default_sample_degrees() -> list[float]:
    """默认采样：全周 1°，并在 260°~280° 加密到 0.1°，跳过 180°。"""
    values: set[float] = set()
    for angle in range(0, 361):
        if angle == int(SINGULARITY_DEGREES):
            continue
        values.add(round(float(angle), 10))

    for value in np.arange(260.0, 280.0 + 0.05, 0.1):
        rounded = round(float(value), 10)
        if abs(rounded - SINGULARITY_DEGREES) < 1e-9:
            continue
        values.add(rounded)

    return sorted(values)


def load_reference_rows(path: Path = REFERENCE_CSV) -> list[dict[str, float]]:
    """读取旧的 Joint2 数值主分支，仅用于最后趋势对比。"""
    if not path.exists():
        return []
    with path.open("r", encoding="utf-8", newline="") as file:
        reader = csv.DictReader(file)
        return [{key: float(value) for key, value in row.items()} for row in reader]


def unwrap_radians(values: Iterable[float]) -> np.ndarray:
    """把角度序列展开成连续曲线。"""
    array = np.array(list(values), dtype=float)
    return np.unwrap(array)


def compare_trend_with_reference(selected_rows: list[dict[str, float | int | str]]) -> dict[str, object]:
    """只做最后的 Fourier/数值延拓趋势对比，不参与选根。"""
    reference_rows = load_reference_rows()
    if not reference_rows or not selected_rows:
        return {"reference_available": False, "trend_consistent": False, "joint_metrics": {}}

    selected_success = [row for row in selected_rows if bool(row["success"])]
    if len(selected_success) < 3:
        return {"reference_available": True, "trend_consistent": False, "joint_metrics": {}}

    selected_deg = np.array([float(row["s_deg"]) for row in selected_success], dtype=float)
    reference_deg = np.array([row["J2_Drive_Angle_deg"] for row in reference_rows], dtype=float)
    joint_mapping = {
        "delta_theta1_rad": "J1_Angle_deg",
        "delta_theta3_rad": "J3_Angle_deg",
        "delta_theta4_rad": "J4_Angle_deg",
    }

    metrics: dict[str, dict[str, float]] = {}
    consistent_flags: list[bool] = []
    for selected_key, reference_key in joint_mapping.items():
        selected_curve = unwrap_radians(float(row[selected_key]) for row in selected_success)
        selected_curve_deg = np.degrees(selected_curve - selected_curve[0])

        reference_curve_rad = np.radians(np.array([row[reference_key] for row in reference_rows], dtype=float))
        reference_curve_deg = np.degrees(unwrap_radians(reference_curve_rad) - unwrap_radians(reference_curve_rad)[0])
        reference_interp = np.interp(selected_deg, reference_deg, reference_curve_deg)

        if np.std(selected_curve_deg) < 1e-12 or np.std(reference_interp) < 1e-12:
            correlation = 1.0 if np.allclose(selected_curve_deg, reference_interp, atol=1e-9) else 0.0
        else:
            correlation = float(np.corrcoef(selected_curve_deg, reference_interp)[0, 1])

        mae = float(np.mean(np.abs(selected_curve_deg - reference_interp)))
        metrics[selected_key] = {"correlation": correlation, "centered_mae_deg": mae}
        consistent_flags.append(correlation >= 0.8)

    return {
        "reference_available": True,
        "trend_consistent": all(consistent_flags),
        "joint_metrics": metrics,
    }


def write_selected_csv(rows: list[dict[str, float | int | str]], output_dir: Path) -> Path:
    """导出连续跟踪后选中的 branch。"""
    path = output_dir / "joint2_branch_selected_continuous.csv"
    fieldnames = [
        "sample_index",
        "success",
        "s_deg",
        "s_rad",
        "q",
        "Y",
        "X",
        "Z",
        "root_rank",
        "xz_rank",
        "root_key",
        "branch_id",
        "psi_residual",
        "closure_residual",
        "continuity_error",
        "predictor_error",
        "priority_group",
        "selection_score",
        "score",
        *JOINT_KEYS,
        "delta_theta1_deg",
        "delta_theta2_deg",
        "delta_theta3_deg",
        "delta_theta4_deg",
        "delta_theta5_deg",
        "delta_theta6_deg",
        "delta_theta7_deg",
    ]
    with path.open("w", encoding="utf-8", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    return path


def write_candidates_csv(rows: list[dict[str, float | int | str]], output_dir: Path) -> Path:
    """导出 260°~280° 范围内的全部候选根评分。"""
    path = output_dir / "joint2_branch_candidates_260_280.csv"
    fieldnames = [
        "s_deg",
        "branch_id",
        "delta_theta1_deg",
        "delta_theta3_deg",
        "delta_theta4_deg",
        "closure_residual",
        "continuity_error",
        "predictor_error",
        "priority_group",
        "selection_score",
        "score",
    ]
    with path.open("w", encoding="utf-8", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    return path


def plot_angles(rows: list[dict[str, float | int | str]], output_dir: Path) -> Path:
    """绘制连续跟踪后的关节角曲线。"""
    success_rows = [row for row in rows if bool(row["success"])]
    x_axis = [float(row["s_deg"]) for row in success_rows]

    fig, ax = plt.subplots(figsize=(11, 6))
    ax.plot(x_axis, [float(row["delta_theta1_deg"]) for row in success_rows], label="Delta theta1", linewidth=2.0)
    ax.plot(x_axis, [float(row["delta_theta3_deg"]) for row in success_rows], label="Delta theta3", linewidth=2.0)
    ax.plot(x_axis, [float(row["delta_theta4_deg"]) for row in success_rows], label="Delta theta4", linewidth=2.0)
    ax.plot(x_axis, [float(row["delta_theta5_deg"]) for row in success_rows], label="Delta theta5", linewidth=1.5, linestyle="--")
    ax.plot(x_axis, [float(row["delta_theta6_deg"]) for row in success_rows], label="Delta theta6", linewidth=1.5, linestyle="--")
    ax.plot(x_axis, [float(row["delta_theta7_deg"]) for row in success_rows], label="Delta theta7", linewidth=1.5, linestyle="--")
    ax.set_xlabel("s (deg)")
    ax.set_ylabel("Joint angle (deg)")
    ax.set_title("Continuous selected Joint2 branch angles")
    ax.grid(True, alpha=0.3)
    ax.legend(ncol=2)

    path = output_dir / "joint2_branch_angles_continuous.png"
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)
    return path


def plot_residual(rows: list[dict[str, float | int | str]], output_dir: Path) -> Path:
    """绘制连续跟踪后的完整闭环残差曲线。"""
    success_rows = [row for row in rows if bool(row["success"])]
    x_axis = [float(row["s_deg"]) for row in success_rows]
    residuals = [max(float(row["closure_residual"]), 1e-18) for row in success_rows]

    fig, ax = plt.subplots(figsize=(11, 4.8))
    ax.semilogy(x_axis, residuals, linewidth=2.0, color="#b33939")
    ax.set_xlabel("s (deg)")
    ax.set_ylabel("||T-I||_F")
    ax.set_title("Continuous selected Joint2 branch residual")
    ax.grid(True, which="both", alpha=0.3)

    path = output_dir / "joint2_branch_residual_continuous.png"
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)
    return path


def summarize_switches(rows: list[dict[str, float | int | str]]) -> tuple[list[dict[str, float | int | str]], bool]:
    """汇总 branch_id 变化点，并判定是错选还是代数重编号。"""
    success_rows = [row for row in rows if bool(row["success"])]
    switch_rows: list[dict[str, float | int | str]] = []
    has_discontinuous_switch = False

    for previous, current in zip(success_rows[:-1], success_rows[1:]):
        if str(previous["branch_id"]) == str(current["branch_id"]):
            continue
        continuity = float(current["continuity_error"])
        continuous = continuity <= SWITCH_CONTINUITY_THRESHOLD
        if not continuous:
            has_discontinuous_switch = True
        switch_rows.append(
            {
                "s_prev_deg": float(previous["s_deg"]),
                "s_curr_deg": float(current["s_deg"]),
                "branch_prev": str(previous["branch_id"]),
                "branch_curr": str(current["branch_id"]),
                "continuity_error": continuity,
                "classification": "algebraic_renumbering" if continuous else "wrong_root_selection",
            }
        )

    return switch_rows, has_discontinuous_switch


def write_report(summary: dict[str, object], output_dir: Path) -> Path:
    """输出新的连续跟踪报告。"""
    trend = summary["trend"]
    joint_metrics = trend["joint_metrics"]
    switch_rows = summary["switch_rows"]
    lines = [
        "Joint2 continuous branch tracking report",
        f"sample_count={summary['sample_count']}",
        f"successful_points={summary['successful_points']}",
        f"failed_points={len(summary['failed_points'])}",
        f"failed_s_deg={summary['failed_points']}",
        f"max_closure_residual={summary['max_closure_residual']}",
        f"mean_closure_residual={summary['mean_closure_residual']}",
        f"branch_switch_detected={summary['branch_switch_detected']}",
        f"branch_switch_count={summary['branch_switch_count']}",
        f"passes_initial_configuration={summary['passes_initial_configuration']}",
        f"reference_available={trend['reference_available']}",
        f"trend_consistent_with_fourier_numeric_continuation={trend['trend_consistent']}",
        "",
        "Scoring rule",
        f"closure_residual_weight={CLOSURE_RESIDUAL_WEIGHT}",
        f"continuity_weight={CONTINUITY_WEIGHT}",
        f"predictor_weight={PREDICTOR_WEIGHT}",
        f"residual_priority_threshold={RESIDUAL_PRIORITY_THRESHOLD}",
        "",
        "Switch interpretation",
        f"switch_continuity_threshold={SWITCH_CONTINUITY_THRESHOLD}",
        "如果切换点前后关节角不连续，则原切换是错误选根；如果连续，则只是代数根编号变化。",
        "",
        "Switch details",
    ]

    if not switch_rows:
        lines.append("No branch-id switch detected.")
    else:
        for item in switch_rows:
            lines.append(
                f"s_prev={item['s_prev_deg']}, s_curr={item['s_curr_deg']}, "
                f"branch_prev={item['branch_prev']}, branch_curr={item['branch_curr']}, "
                f"continuity_error={item['continuity_error']}, classification={item['classification']}"
            )

    lines.extend(["", "Trend comparison metrics"])
    for joint_name, metrics in joint_metrics.items():
        lines.append(
            f"{joint_name}: correlation={metrics['correlation']}, centered_mae_deg={metrics['centered_mae_deg']}"
        )

    lines.extend(
        [
            "",
            "Trend note",
            "Reference data comes from the existing q2 numerical/Fourier workflow and is used only for final trend comparison.",
        ]
    )

    path = output_dir / "joint2_branch_switch_report.txt"
    path.write_text("\n".join(lines), encoding="utf-8")
    return path


def trace_selected_branch(
    sample_degrees: Iterable[float] | None = None,
    output_dir: Path | str = OUTPUT_DIR,
) -> dict[str, object]:
    """主流程：强连续跟踪 Joint2 解析分支，并导出 continuous 结果。"""
    if sample_degrees is None:
        sample_degrees = default_sample_degrees()

    sample_list = [float(value) for value in sample_degrees]
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)

    rows: list[dict[str, float | int | str]] = []
    failed_points: list[float] = []
    candidate_rows_260_280: list[dict[str, float | int | str]] = []
    previous_selected: dict[str, float | int | str] | None = None
    previous_selected2: dict[str, float | int | str] | None = None

    for sample_index, s_deg in enumerate(sample_list):
        candidates = build_candidate_rows_for_sample(s_deg)

        if previous_selected is None:
            selected = choose_initial_candidate(candidates)
            scored_candidates = []
            for candidate in candidates:
                enriched = dict(candidate)
                enriched["continuity_error"] = 0.0
                enriched["predictor_error"] = 0.0
                enriched["selection_score"] = zero_pose_error(candidate)
                scored_candidates.append(enriched)
            if selected is not None:
                selected = dict(selected)
                selected["continuity_error"] = 0.0
                selected["predictor_error"] = 0.0
                selected["selection_score"] = zero_pose_error(selected)
        else:
            selected, scored_candidates = choose_continuous_candidate(candidates, previous_selected, previous_selected2)

        if 260.0 <= s_deg <= 280.0:
            for candidate in scored_candidates:
                candidate_rows_260_280.append(
                    {
                        "s_deg": candidate["s_deg"],
                        "branch_id": candidate["branch_id"],
                        "delta_theta1_deg": math.degrees(float(candidate["delta_theta1_rad"])),
                        "delta_theta3_deg": math.degrees(float(candidate["delta_theta3_rad"])),
                        "delta_theta4_deg": math.degrees(float(candidate["delta_theta4_rad"])),
                        "closure_residual": float(candidate["closure_residual"]),
                        "continuity_error": float(candidate.get("continuity_error", 0.0)),
                        "predictor_error": float(candidate.get("predictor_error", 0.0)),
                        "selection_score": float(candidate.get("selection_score", 0.0)),
                    }
                )

        if selected is None:
            failed_points.append(s_deg)
            rows.append(
                {
                    "sample_index": sample_index,
                    "success": False,
                    "s_deg": s_deg,
                    "s_rad": math.radians(s_deg),
                    "q": half_angle_input_from_s(math.radians(s_deg)),
                    "Y": float("nan"),
                    "X": float("nan"),
                    "Z": float("nan"),
                    "root_rank": -1,
                    "xz_rank": -1,
                    "root_key": "failed",
                    "branch_id": "failed",
                    "psi_residual": float("nan"),
                    "closure_residual": float("nan"),
                    "continuity_error": float("nan"),
                    "predictor_error": float("nan"),
                    "selection_score": float("nan"),
                    **{key: float("nan") for key in JOINT_KEYS},
                    "delta_theta1_deg": float("nan"),
                    "delta_theta2_deg": float("nan"),
                    "delta_theta3_deg": float("nan"),
                    "delta_theta4_deg": float("nan"),
                    "delta_theta5_deg": float("nan"),
                    "delta_theta6_deg": float("nan"),
                    "delta_theta7_deg": float("nan"),
                }
            )
            continue

        current_row = {
            "sample_index": sample_index,
            "success": True,
            **selected,
            "delta_theta1_deg": math.degrees(float(selected["delta_theta1_rad"])),
            "delta_theta2_deg": math.degrees(float(selected["delta_theta2_rad"])),
            "delta_theta3_deg": math.degrees(float(selected["delta_theta3_rad"])),
            "delta_theta4_deg": math.degrees(float(selected["delta_theta4_rad"])),
            "delta_theta5_deg": math.degrees(float(selected["delta_theta5_rad"])),
            "delta_theta6_deg": math.degrees(float(selected["delta_theta6_rad"])),
            "delta_theta7_deg": math.degrees(float(selected["delta_theta7_rad"])),
        }
        rows.append(current_row)
        previous_selected2 = previous_selected
        previous_selected = current_row

    write_selected_csv(rows, output_path)
    write_candidates_csv(candidate_rows_260_280, output_path)
    plot_angles(rows, output_path)
    plot_residual(rows, output_path)

    success_rows = [row for row in rows if bool(row["success"])]
    switch_rows, has_discontinuous_switch = summarize_switches(rows)
    trend = compare_trend_with_reference(rows)
    summary = {
        "sample_count": len(rows),
        "successful_points": len(success_rows),
        "failed_points": failed_points,
        "max_closure_residual": max((float(row["closure_residual"]) for row in success_rows), default=float("nan")),
        "mean_closure_residual": float(np.mean([float(row["closure_residual"]) for row in success_rows])) if success_rows else float("nan"),
        "branch_switch_detected": len(switch_rows) > 0,
        "branch_switch_count": len(switch_rows),
        "has_discontinuous_switch": has_discontinuous_switch,
        "passes_initial_configuration": bool(
            success_rows
            and abs(float(success_rows[0]["delta_theta1_rad"])) < 1e-9
            and abs(float(success_rows[0]["delta_theta3_rad"])) < 1e-9
            and abs(float(success_rows[0]["delta_theta4_rad"])) < 1e-9
            and float(success_rows[0]["closure_residual"]) < 1e-12
        ),
        "trend": trend,
        "switch_rows": switch_rows,
    }
    write_report(summary, output_path)
    return {
        "selected_rows": rows,
        "failed_points": failed_points,
        "summary": summary,
        "candidate_rows_260_280": candidate_rows_260_280,
    }


def main() -> None:
    """命令行入口。"""
    result = trace_selected_branch()
    print(f"Output directory: {OUTPUT_DIR}")
    print(f"Successful points: {result['summary']['successful_points']}/{result['summary']['sample_count']}")
    print(f"Max closure residual: {result['summary']['max_closure_residual']}")
    print(f"Branch switch detected: {result['summary']['branch_switch_detected']}")


if __name__ == "__main__":
    main()
