"""
最小 MDH 闭环核查脚本。

目的：
1. 严格按照项目要求的 MDH 形式构造 7 个齐次变换；
2. 检查初始位形是否满足 T = I；
3. 读取已有数值延拓 CSV，逐点代入完整 MDH，重新计算 ||T - I||_F；
4. 在基础定义未确认前，不做任何消元或主分支处理。
"""

from __future__ import annotations

import csv
import math
import time
from pathlib import Path
from typing import Iterable

import numpy as np


PROJECT_DIR = Path(__file__).resolve().parent
DEFAULT_INPUT_CSV = PROJECT_DIR / "joint_solution_numeric.csv"
DEFAULT_OUTPUT_CSV = PROJECT_DIR / "mdh_closure_check_against_numeric.csv"
DEFAULT_REPORT_TXT = PROJECT_DIR / "mdh_closure_check_report.txt"


def log(message: str) -> None:
    """打印带时间戳的进度，避免长时间无输出。"""
    now = time.strftime("%H:%M:%S")
    print(f"[{now}] {message}", flush=True)


def rx(alpha: float) -> np.ndarray:
    """绕 x 轴旋转的 4x4 齐次矩阵。"""
    ca = math.cos(alpha)
    sa = math.sin(alpha)
    return np.array(
        [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, ca, -sa, 0.0],
            [0.0, sa, ca, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


def tx(a_value: float) -> np.ndarray:
    """沿 x 轴平移的 4x4 齐次矩阵。"""
    return np.array(
        [
            [1.0, 0.0, 0.0, a_value],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


def rz(theta: float) -> np.ndarray:
    """绕 z 轴旋转的 4x4 齐次矩阵。"""
    ct = math.cos(theta)
    st = math.sin(theta)
    return np.array(
        [
            [ct, -st, 0.0, 0.0],
            [st, ct, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


def tz(d_value: float) -> np.ndarray:
    """沿 z 轴平移的 4x4 齐次矩阵。"""
    return np.array(
        [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, d_value],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


def mdh_matrix(alpha_prev: float, a_prev: float, d_i: float, theta_i: float) -> np.ndarray:
    """
    严格使用用户给定的 modified DH 公式：
    A_i = Rx(alpha_{i-1}) * Tx(a_{i-1}) * Rz(theta_i) * Tz(d_i)
    """
    return rx(alpha_prev) @ tx(a_prev) @ rz(theta_i) @ tz(d_i)


def build_transforms(
    theta1: float,
    theta2: float,
    theta3: float,
    theta4: float,
    theta5: float,
    theta6: float,
    theta7: float,
    length_scale: float = 1.0,
) -> list[np.ndarray]:
    """按题目给定的 7 组 MDH 参数顺序生成 A1...A7。"""
    l_value = float(length_scale)
    return [
        mdh_matrix(0.0, l_value, 0.0, theta1),
        mdh_matrix(math.pi / 2.0, 2.0 * l_value, 2.5 * l_value, theta2),
        mdh_matrix(math.pi / 2.0, 0.0, 0.0, theta3),
        mdh_matrix(math.pi / 2.0, 2.5 * l_value, 0.0, theta4),
        mdh_matrix(math.pi / 2.0, 2.5 * l_value, 0.0, theta5),
        mdh_matrix(math.pi / 2.0, 0.0, 2.5 * l_value, theta6),
        mdh_matrix(math.pi / 2.0, 2.0 * l_value, 0.0, theta7),
    ]


def build_total_transform(
    theta1: float,
    theta2: float,
    theta3: float,
    theta4: float,
    theta5: float,
    theta6: float,
    theta7: float,
    length_scale: float = 1.0,
) -> np.ndarray:
    """顺序相乘得到 T = A1*A2*...*A7。"""
    total = np.eye(4, dtype=float)
    for transform in build_transforms(theta1, theta2, theta3, theta4, theta5, theta6, theta7, length_scale=length_scale):
        total = total @ transform
    return total


def closure_norm(total: np.ndarray, target: np.ndarray | None = None) -> float:
    """计算与目标位形的 Frobenius 范数，默认目标是单位阵。"""
    if target is None:
        target = np.eye(4, dtype=float)
    return float(np.linalg.norm(total - target, ord="fro"))


def initial_configuration_transform(length_scale: float = 1.0) -> np.ndarray:
    """题目给定初始位形：theta1=0, theta2=0, theta3=pi, theta4=0, theta5=0, theta6=pi, theta7=0。"""
    return build_total_transform(
        theta1=0.0,
        theta2=0.0,
        theta3=math.pi,
        theta4=0.0,
        theta5=0.0,
        theta6=math.pi,
        theta7=0.0,
        length_scale=length_scale,
    )


def pretty_matrix(matrix: np.ndarray) -> str:
    """把矩阵格式化成易读文本。"""
    return np.array2string(
        matrix,
        precision=15,
        suppress_small=False,
        floatmode="maxprec",
    )


def _pick_numeric_column(row: dict[str, str], preferred: Iterable[str]) -> float:
    """按优先列名读取浮点值，用于兼容不同 CSV 字段名。"""
    for key in preferred:
        if key in row and row[key] != "":
            return float(row[key])
    raise KeyError(f"Missing any of columns: {preferred}")


def load_numeric_rows(csv_path: Path) -> list[dict[str, str]]:
    """读取数值延拓 CSV 全部行。"""
    with csv_path.open("r", encoding="utf-8", newline="") as file:
        reader = csv.DictReader(file)
        return list(reader)


def audit_numeric_csv(
    input_csv: Path = DEFAULT_INPUT_CSV,
    output_csv: Path = DEFAULT_OUTPUT_CSV,
    report_txt: Path = DEFAULT_REPORT_TXT,
    length_scale: float = 1.0,
) -> dict[str, float | int]:
    """
    逐点代入已有 CSV 中的角度，使用完整 MDH 重新计算 T 与 ||T-I||_F。
    不相信原 CSV 自带残差，全部重算。
    """
    log(f"Reading numeric continuation CSV: {input_csv}")
    rows = load_numeric_rows(input_csv)
    if not rows:
        raise ValueError(f"CSV is empty: {input_csv}")

    output_rows: list[dict[str, float | str | int]] = []
    max_norm = -1.0
    max_index = -1
    max_s_deg = float("nan")

    for index, row in enumerate(rows):
        s_rad = _pick_numeric_column(row, ("s_rad",))
        x_rad = _pick_numeric_column(row, ("x_rad", "theta1_rad"))
        y_rad = _pick_numeric_column(row, ("y_rad", "theta2_rad"))
        z_rad = _pick_numeric_column(row, ("z_rad", "theta4_rad"))

        total = build_total_transform(
            theta1=x_rad,
            theta2=y_rad,
            theta3=math.pi + s_rad,
            theta4=z_rad,
            theta5=-s_rad,
            theta6=math.pi - y_rad,
            theta7=x_rad,
            length_scale=length_scale,
        )
        fro_norm = closure_norm(total)
        if fro_norm > max_norm:
            max_norm = fro_norm
            max_index = index
            max_s_deg = _pick_numeric_column(row, ("s_deg",))

        output_rows.append(
            {
                "sample_index": int(float(row.get("sample_index", index))),
                "s_deg": _pick_numeric_column(row, ("s_deg",)),
                "s_rad": s_rad,
                "x_rad": x_rad,
                "y_rad": y_rad,
                "z_rad": z_rad,
                "x_deg": math.degrees(x_rad),
                "y_deg": math.degrees(y_rad),
                "z_deg": math.degrees(z_rad),
                "mdh_closure_frobenius_norm": fro_norm,
                "original_frobenius_residual": row.get("frobenius_residual", ""),
                "original_solver_method": row.get("solver_method", ""),
            }
        )

        if index == 0 or (index + 1) % 30 == 0 or (index + 1) == len(rows):
            log(
                f"Checked {index + 1}/{len(rows)} points, "
                f"s={_pick_numeric_column(row, ('s_deg',)):.1f} deg, "
                f"||T-I||F={fro_norm:.6e}"
            )

    log(f"Writing closure audit CSV: {output_csv}")
    with output_csv.open("w", encoding="utf-8", newline="") as file:
        writer = csv.DictWriter(
            file,
            fieldnames=[
                "sample_index",
                "s_deg",
                "s_rad",
                "x_rad",
                "y_rad",
                "z_rad",
                "x_deg",
                "y_deg",
                "z_deg",
                "mdh_closure_frobenius_norm",
                "original_frobenius_residual",
                "original_solver_method",
            ],
        )
        writer.writeheader()
        writer.writerows(output_rows)

    metrics = {
        "point_count": len(output_rows),
        "max_norm": max_norm,
        "mean_norm": float(np.mean([float(item["mdh_closure_frobenius_norm"]) for item in output_rows])),
        "std_norm": float(np.std([float(item["mdh_closure_frobenius_norm"]) for item in output_rows])),
        "max_index": max_index,
        "max_s_deg": max_s_deg,
    }

    report_lines = [
        "MDH closure basic audit",
        f"input_csv={input_csv}",
        f"output_csv={output_csv}",
        f"point_count={metrics['point_count']}",
        f"max_norm={metrics['max_norm']}",
        f"mean_norm={metrics['mean_norm']}",
        f"std_norm={metrics['std_norm']}",
        f"max_index={metrics['max_index']}",
        f"max_s_deg={metrics['max_s_deg']}",
        "",
        "If these norms are not near zero, stop symbolic elimination and re-check:",
        "- MDH multiplication order",
        "- closure target T=I or T=T0",
        "- theta0 offsets duplicated or omitted",
        "- angle unit mismatch",
        "- standard DH vs modified DH mix-up",
    ]
    report_txt.write_text("\n".join(report_lines), encoding="utf-8")
    return metrics


def main() -> None:
    log("Checking initial MDH configuration")
    initial_total = initial_configuration_transform(length_scale=1.0)
    initial_norm = closure_norm(initial_total)

    print("Initial configuration T =")
    print(pretty_matrix(initial_total))
    print(f"Initial ||T - I||_F = {initial_norm:.16e}")

    metrics = audit_numeric_csv()
    print("")
    print("Numeric continuation audit summary")
    print(f"point_count = {metrics['point_count']}")
    print(f"max ||T-I||_F = {metrics['max_norm']:.16e}")
    print(f"mean ||T-I||_F = {metrics['mean_norm']:.16e}")
    print(f"std ||T-I||_F = {metrics['std_norm']:.16e}")
    print(f"max residual at s_deg = {metrics['max_s_deg']}")

    print("")
    print("Stop conditions / interpretation")
    print("- 如果初始位形不满足 T=I，优先怀疑闭环目标不是 I，而是相对初始位形 T0。")
    print("- 如果初始位形满足 T=I，但 CSV 逐点不满足，优先怀疑数值延拓结果本身不在完整 MDH 闭环上。")
    print("- 在这些基础问题没确认前，不应继续任何符号消元。")


if __name__ == "__main__":
    main()
