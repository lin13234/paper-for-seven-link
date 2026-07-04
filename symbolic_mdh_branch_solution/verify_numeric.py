"""
Numeric continuation-based verification for the 7R closed-chain MDH model.

This script:
1. Uses the full MDH closure matrix.
2. Samples s from 0 to 360 degrees at 361 points.
3. Solves x, y, z continuously with least_squares.
4. Saves the Frobenius closure residual for each sample.
5. Falls back to translation plus vee((R-R^T)/2) residuals when needed.
"""

import csv
import math
import time
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import least_squares


OUTPUT_DIR = Path(__file__).resolve().parent
CSV_PATH = OUTPUT_DIR / "joint_solution_numeric.csv"
PLOT_PATH = OUTPUT_DIR / "joint_angles_vs_delta_theta3.png"

SAMPLE_COUNT = 361
FROBENIUS_THRESHOLD = 1e-8
SOLVER_MAX_NFEV = 400


def log(message: str) -> None:
    now = time.strftime("%H:%M:%S")
    print(f"[{now}] {message}", flush=True)


def rx(alpha: float) -> np.ndarray:
    ca = math.cos(alpha)
    sa = math.sin(alpha)
    return np.array([
        [1.0, 0.0, 0.0, 0.0],
        [0.0, ca, -sa, 0.0],
        [0.0, sa, ca, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ], dtype=float)


def tx(a_value: float) -> np.ndarray:
    return np.array([
        [1.0, 0.0, 0.0, a_value],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ], dtype=float)


def rz(theta: float) -> np.ndarray:
    ct = math.cos(theta)
    st = math.sin(theta)
    return np.array([
        [ct, -st, 0.0, 0.0],
        [st, ct, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ], dtype=float)


def tz(d_value: float) -> np.ndarray:
    return np.array([
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, d_value],
        [0.0, 0.0, 0.0, 1.0],
    ], dtype=float)


def mdh_matrix(alpha_prev: float, a_prev: float, d_i: float, theta_i: float) -> np.ndarray:
    return rx(alpha_prev) @ tx(a_prev) @ rz(theta_i) @ tz(d_i)


def closure_matrix(variables: np.ndarray, s_value: float, length_scale: float = 1.0) -> np.ndarray:
    x_value, y_value, z_value = variables
    transforms = [
        mdh_matrix(0.0, length_scale, 0.0, x_value),
        mdh_matrix(math.pi / 2.0, 2.0 * length_scale, 2.5 * length_scale, y_value),
        mdh_matrix(math.pi / 2.0, 0.0, 0.0, math.pi + s_value),
        mdh_matrix(math.pi / 2.0, 2.5 * length_scale, 0.0, z_value),
        mdh_matrix(math.pi / 2.0, 2.5 * length_scale, 0.0, -s_value),
        mdh_matrix(math.pi / 2.0, 0.0, 2.5 * length_scale, math.pi - y_value),
        mdh_matrix(math.pi / 2.0, 2.0 * length_scale, 0.0, x_value),
    ]
    total = np.eye(4, dtype=float)
    for transform in transforms:
        total = total @ transform
    return total


def full_matrix_residual(variables: np.ndarray, s_value: float) -> np.ndarray:
    delta = closure_matrix(variables, s_value) - np.eye(4, dtype=float)
    return delta.reshape(-1)


def pose_residual(variables: np.ndarray, s_value: float) -> np.ndarray:
    total = closure_matrix(variables, s_value)
    rotation = total[:3, :3]
    translation = total[:3, 3]
    skew = 0.5 * (rotation - rotation.T)
    vee = np.array([skew[2, 1], skew[0, 2], skew[1, 0]], dtype=float)
    return np.concatenate([translation, vee])


def solve_one_step(s_value: float, initial_guess: np.ndarray) -> tuple[np.ndarray, float, str, int, float]:
    primary = least_squares(
        fun=full_matrix_residual,
        x0=initial_guess,
        args=(s_value,),
        max_nfev=SOLVER_MAX_NFEV,
        xtol=1e-12,
        ftol=1e-12,
        gtol=1e-12,
    )
    primary_fro = float(np.linalg.norm(closure_matrix(primary.x, s_value) - np.eye(4), ord="fro"))
    if primary.success and primary_fro < FROBENIUS_THRESHOLD:
        return primary.x, primary_fro, "full_matrix", primary.nfev, float(primary.cost)

    fallback = least_squares(
        fun=pose_residual,
        x0=primary.x,
        args=(s_value,),
        max_nfev=SOLVER_MAX_NFEV,
        xtol=1e-12,
        ftol=1e-12,
        gtol=1e-12,
    )
    fallback_fro = float(np.linalg.norm(closure_matrix(fallback.x, s_value) - np.eye(4), ord="fro"))
    if fallback_fro < primary_fro:
        return fallback.x, fallback_fro, "pose_residual", fallback.nfev, float(fallback.cost)
    return primary.x, primary_fro, "full_matrix_poor", primary.nfev, float(primary.cost)


def main() -> None:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    log("Starting numeric verification with 361 samples")

    s_values_deg = np.linspace(0.0, 360.0, SAMPLE_COUNT)
    s_values_rad = np.deg2rad(s_values_deg)
    rows: list[dict[str, float | str]] = []
    current_guess = np.array([0.0, 0.0, 0.0], dtype=float)

    for index, (s_deg, s_rad) in enumerate(zip(s_values_deg, s_values_rad), start=1):
        solution, fro_norm, method, nfev, cost = solve_one_step(s_rad, current_guess)
        current_guess = solution
        rows.append({
            "sample_index": index - 1,
            "s_deg": float(s_deg),
            "s_rad": float(s_rad),
            "x_rad": float(solution[0]),
            "y_rad": float(solution[1]),
            "z_rad": float(solution[2]),
            "x_deg": float(np.rad2deg(solution[0])),
            "y_deg": float(np.rad2deg(solution[1])),
            "z_deg": float(np.rad2deg(solution[2])),
            "frobenius_residual": fro_norm,
            "solver_method": method,
            "nfev": int(nfev),
            "cost": float(cost),
        })
        if index == 1 or index % 20 == 0 or index == SAMPLE_COUNT:
            log(f"Sample {index}/{SAMPLE_COUNT}, s={s_deg:.1f} deg, method={method}, fro={fro_norm:.3e}")

    log("Writing CSV output")
    with CSV_PATH.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=[
            "sample_index",
            "s_deg",
            "s_rad",
            "x_rad",
            "y_rad",
            "z_rad",
            "x_deg",
            "y_deg",
            "z_deg",
            "frobenius_residual",
            "solver_method",
            "nfev",
            "cost",
        ])
        writer.writeheader()
        writer.writerows(rows)

    log("Rendering angle plot")
    fig, ax = plt.subplots(figsize=(10, 6))
    ax.plot(s_values_deg, [row["x_deg"] for row in rows], label="x = Delta theta1 (deg)", linewidth=1.5)
    ax.plot(s_values_deg, [row["y_deg"] for row in rows], label="y = Delta theta2 (deg)", linewidth=1.5)
    ax.plot(s_values_deg, [row["z_deg"] for row in rows], label="z = Delta theta4 (deg)", linewidth=1.5)
    ax.set_xlabel("s = Delta theta3 (deg)")
    ax.set_ylabel("Joint angle (deg)")
    ax.set_title("Joint angles vs Delta theta3 for the 7R closed chain")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(PLOT_PATH, dpi=200)
    plt.close(fig)

    log(f"Saved CSV to {CSV_PATH.name}")
    log(f"Saved plot to {PLOT_PATH.name}")
    log("Numeric verification finished")


if __name__ == "__main__":
    main()
