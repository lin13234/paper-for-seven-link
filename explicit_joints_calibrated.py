import os
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt


def explicit_kinematics_solver(theta1_array, L1, L2, L3, L4, branch=1):
    """
    四杆平面机构的解析求解器（零位补偿版）。

    参数
    ----
    theta1_array : ndarray
        驱动关节输入角数组，单位：rad。
    L1, L2, L3, L4 : float
        四杆机构杆长参数。
    branch : int
        装配支路选择，+1 或 -1。不同支路会给出不同的闭链解。

    返回
    ----
    t2, t4, t6 : ndarray
        经过零位补偿和 unwrap 展开的被动关节变化量，单位：rad。

    说明
    ----
    这段程序的目标不是改变机构解析式，而是把“绝对角”整理成
    “从初始位形开始的相对变化量”，这样更适合放进论文里的曲线图。
    """

    theta1_array = np.asarray(theta1_array, dtype=float)
    if branch not in (+1, -1):
        raise ValueError("branch 只能取 +1 或 -1")

    def get_raw_angles(theta1):
        """
        计算未做零位补偿的原始角度。

        返回的三个角对应你当前程序里的：
        - t2：关节2相对变化
        - t4：关节4相对变化
        - t6：关节6相对变化
        """
        A = 2.0 * L4 * (L1 - L2 * np.cos(theta1))
        B = -2.0 * L4 * L2 * np.sin(theta1)
        C = L1**2 + L2**2 + L4**2 - L3**2 - 2.0 * L1 * L2 * np.cos(theta1)
        delta = B**2 - C**2 + A**2

        # 数值误差下 delta 可能出现极小负值，截断到 0，避免 sqrt 出现 NaN。
        delta = max(delta, 0.0)
        den = C - A

        # 分支解：branch=+1 / -1
        t = (-B + branch * np.sqrt(delta)) / den
        a3 = 2.0 * np.arctan(t)

        a2 = np.arctan2(
            L4 * np.sin(a3) - L2 * np.sin(theta1),
            L1 + L4 * np.cos(a3) - L2 * np.cos(theta1),
        )

        # 这里保留你原来的三角关系定义，不擅自改机构拓扑。
        t2 = a2 - theta1
        t4 = a3 - a2
        t6 = (2.0 * np.pi - a3) - 2.0 * np.pi
        return t2, t4, t6

    # 零位补偿：以 theta1 = 0 时的输出作为基准，保证起点从 0 开始
    off2, off4, off6 = get_raw_angles(0.0)

    alpha2_array = np.zeros_like(theta1_array)
    alpha3_array = np.zeros_like(theta1_array)

    A = 2.0 * L4 * (L1 - L2 * np.cos(theta1_array))
    B = -2.0 * L4 * L2 * np.sin(theta1_array)
    C = L1**2 + L2**2 + L4**2 - L3**2 - 2.0 * L1 * L2 * np.cos(theta1_array)
    delta = B**2 - C**2 + A**2

    # 只在可解区域求解；不可解区域保持为 0，避免程序崩掉。
    valid_mask = delta >= 0

    if np.any(valid_mask):
        delta_valid = np.clip(delta[valid_mask], 0.0, None)
        den_valid = C[valid_mask] - A[valid_mask]

        # 避免分母接近 0 的点直接炸掉。
        small_den_mask = np.abs(den_valid) < 1e-12
        den_valid = den_valid.copy()
        den_valid[small_den_mask] = np.nan

        t = (-B[valid_mask] + branch * np.sqrt(delta_valid)) / den_valid
        alpha3_array[valid_mask] = 2.0 * np.arctan(t)

        sin_a2 = (
            L4 * np.sin(alpha3_array[valid_mask]) - L2 * np.sin(theta1_array[valid_mask])
        ) / L3
        cos_a2 = (
            L1 + L4 * np.cos(alpha3_array[valid_mask]) - L2 * np.cos(theta1_array[valid_mask])
        ) / L3
        alpha2_array[valid_mask] = np.arctan2(sin_a2, cos_a2)

    # 零位补偿：把初始位形当成 0，输出相对变化量
    t2 = (alpha2_array - theta1_array) - off2
    t4 = (alpha3_array - alpha2_array) - off4
    t6 = ((2.0 * np.pi - alpha3_array) - 2.0 * np.pi) - off6

    # unwrap 的作用是消掉跨越 ±pi 时出现的跳变，让曲线连续
    return np.unwrap(t2), np.unwrap(t4), np.unwrap(t6)


def main():
    # ==============================
    # 1. 参数设置
    # ==============================
    # 这里的杆长只是示例值，保持与你原程序一致。
    # 如果后续你要换成真实机构尺寸，只需要改这四个数。
    L_ground, L_crank, L_coupler, L_rocker = 4.0, 2.0, 5.0, 4.5
    theta1_vals = np.linspace(0.0, 2.0 * np.pi, 360)

    # ==============================
    # 2. 计算被动关节变化
    # ==============================
    t2, t4, t6 = explicit_kinematics_solver(
        theta1_vals,
        L_ground,
        L_crank,
        L_coupler,
        L_rocker,
        branch=1,
    )

    # 打印零位检查，确认补偿是否生效
    print(
        f"验证: theta1=0 时, "
        f"t2={np.degrees(t2[0]):.4f}°, "
        f"t4={np.degrees(t4[0]):.4f}°, "
        f"t6={np.degrees(t6[0]):.4f}°"
    )

    # ==============================
    # 3. 绘图
    # ==============================
    fig, ax = plt.subplots(figsize=(10, 6), dpi=120)

    ax.plot(np.degrees(theta1_vals), np.degrees(t2), "b-", lw=2, label=r"Joint $\theta_2$")
    ax.plot(np.degrees(theta1_vals), np.degrees(t4), "g-", lw=2, label=r"Joint $\theta_4$")
    ax.plot(np.degrees(theta1_vals), np.degrees(t6), "r-", lw=2, label=r"Joint $\theta_6$")

    ax.set_xlabel(r"Input Driving Joint $\theta_1$ (deg)", fontweight="bold", fontsize=12)
    ax.set_ylabel(r"Passive Joint Angles (deg)", fontweight="bold", fontsize=12)
    ax.set_title(
        r"Calibrated Analytical Solutions (Zeroed at $\theta_1=0$)",
        fontweight="bold",
        fontsize=13,
    )
    ax.legend(loc="best", framealpha=0.9)
    ax.grid(True, linestyle="--", alpha=0.7)
    ax.set_xlim(0, 360)
    fig.tight_layout()

    # ==============================
    # 4. 保存图像
    # ==============================
    out_paths = [
        Path(r"E:\hub\02论文正文\001paper\01_主稿_LaTeX\figs\explicit_joints_calibrated.png"),
        Path(r"E:\hub\02论文正文\001paper\01_主稿_LaTeX\figs\explicit_joints_calibrated.pdf"),
    ]

    for out_path in out_paths:
        out_path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(out_path, dpi=300, bbox_inches="tight")
        print(f"saved: {out_path}")

    plt.close(fig)


if __name__ == "__main__":
    main()
