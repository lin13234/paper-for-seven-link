import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import null_space


def dh_transform(alpha, a, d, theta):
    ca, sa = np.cos(alpha), np.sin(alpha)
    ct, st = np.cos(theta), np.sin(theta)
    return np.array([
        [ct, -st, 0, a],
        [st * ca, ct * ca, -sa, -d * sa],
        [st * sa, ct * sa, ca, d * ca],
        [0, 0, 0, 1]
    ])


def get_kinematics(q, l=1.0):
    params = [
        (0, l, 0, 0),
        (np.pi / 2, 2 * l, 2.5 * l, 0),
        (np.pi / 2, 0, 0, np.pi),
        (np.pi / 2, 2.5 * l, 0, 0),
        (np.pi / 2, 2.5 * l, 0, 0),
        (np.pi / 2, 0, 2.5 * l, np.pi),
        (np.pi / 2, 2 * l, 0, 0)
    ]
    T_acc = [np.eye(4)]
    T_curr = np.eye(4)
    for i in range(7):
        alpha, a, d, offset = params[i]
        T_curr = T_curr @ dh_transform(alpha, a, d, q[i] + offset)
        T_acc.append(T_curr)
    return T_acc


def get_error_and_jacobian(q):
    T_acc = get_kinematics(q)
    T_end = T_acc[-1]

    p_err = T_end[0:3, 3]
    R = T_end[0:3, 0:3]
    w_err = 0.5 * np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    E = np.concatenate((p_err, w_err))

    J = np.zeros((6, 7))
    p_end = T_end[0:3, 3]
    for i in range(7):
        z_i = T_acc[i][0:3, 2]
        p_i = T_acc[i][0:3, 3]
        J[0:3, i] = np.cross(z_i, p_end - p_i)
        J[3:6, i] = z_i
    return E, J


def trace_motion_branch():
    print("--- 1. 获取初始扰动脱离死点 ---")
    q_init = np.zeros(7)
    _, J_init = get_error_and_jacobian(q_init)
    ns = null_space(J_init)
    # 给一个合法的初始扰动，进入运动分支
    q_current = q_init + 0.01 * ns[:, 0]

    # 我们选择 J1 (索引 0) 作为主动件进行测试
    active_joint_idx = 6
    print(f"--- 2. 开始以 J{active_joint_idx + 1} 为主动件，进行轨迹跟踪演化 ---")

    # 让 J1 从当前位置，再向前推进 1.5 弧度 (约 85 度)
    target_q1 = q_current[active_joint_idx] + 1.5
    steps = 100
    q1_trajectory = np.linspace(q_current[active_joint_idx], target_q1, steps)

    sv_history = []

    for q1_val in q1_trajectory:
        q_current[active_joint_idx] = q1_val

        # 牛顿-拉夫逊迭代：锁定 J1，修正剩下的 6 个被动关节，保证机构严丝合缝
        for _ in range(20):
            E, J = get_error_and_jacobian(q_current)
            if np.linalg.norm(E) < 1e-6:
                break
            # 提取 6x6 的被动关节雅可比矩阵
            J_passive = np.delete(J, active_joint_idx, axis=1)
            # 计算被动关节补偿量
            delta_q_passive = np.linalg.pinv(J_passive) @ (-E)

            # 将补偿量加回 q_current 的对应位置
            passive_indices = [i for i in range(7) if i != active_joint_idx]
            q_current[passive_indices] += delta_q_passive

        # 记录收敛后的性能：剔除主动件后的被动矩阵最小奇异值
        _, J_final = get_error_and_jacobian(q_current)
        J_sub = np.delete(J_final, active_joint_idx, axis=1)
        _, S_sub, _ = np.linalg.svd(J_sub)
        sv_history.append(S_sub[-1])

    # --- 3. 绘制结果 ---
    plt.figure(figsize=(10, 6))
    plt.plot(np.degrees(q1_trajectory), sv_history, 'b-', linewidth=2)
    plt.axhline(y=0.1, color='r', linestyle='--', label='Safe Transmission Threshold')

    plt.title(f"Transmission Performance (Min Singular Value) vs J{active_joint_idx + 1} Angle")
    plt.xlabel(f"Driving Joint J{active_joint_idx + 1} Angle (Degrees)")
    plt.ylabel("Min Singular Value of Passive Sub-Jacobian")
    plt.grid(True)
    plt.legend()
    plt.show()


if __name__ == "__main__":
    trace_motion_branch()