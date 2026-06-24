import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize, curve_fit
import pandas as pd
import os


# ==========================================
# 1. 基础运动学定义
# ==========================================
def dh_transform(alpha, a, d, theta):
    ca, sa = np.cos(alpha), np.sin(alpha)
    ct, st = np.cos(theta), np.sin(theta)
    return np.array([
        [ct, -st, 0, a],
        [st * ca, ct * ca, -sa, -d * sa],
        [st * sa, ct * sa, ca, d * ca],
        [0, 0, 0, 1]
    ])


def get_model_data(q, l=1.0,a=2.0,b=2.5):
    # params = [
    #     (0, l, 0, 0),
    #     (np.pi / 2, 2 * l, 2.5 * l, 0),
    #     (np.pi / 2, 0, 0, np.pi),
    #     (np.pi / 2, 2.5 * l, 0, 0),
    #     (np.pi / 2, 2.5 * l, 0, 0),
    #     (np.pi / 2, 0, 2.5 * l, np.pi),
    #     (np.pi / 2, 2 * l, 0, 0)
    # ]
    params = [
        (0, l, 0, 0),
        (np.pi / 2, a, b, 0),
        (np.pi / 2, 0, 0, np.pi),
        (np.pi / 2, (2*a + l) / 2, 0, 0),
        (np.pi / 2, (2*a + l) / 2, 0, 0),
        (np.pi / 2, 0, b, np.pi),
        (np.pi / 2, a, 0, 0)
    ]
    T_curr = np.eye(4)
    T_acc = [T_curr]

    for i in range(7):
        T_curr = T_curr @ dh_transform(params[i][0], params[i][1], params[i][2], q[i] + params[i][3])
        T_acc.append(T_curr)

    T_end = T_acc[-1]
    p_err = T_end[0:3, 3]
    R = T_end[0:3, 0:3]
    w_err = 0.5 * np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    E = np.concatenate((p_err, w_err))

    J = np.zeros((6, 7))
    for i in range(7):
        z_i = T_acc[i][0:3, 2]
        p_i = T_acc[i][0:3, 3]
        J[0:3, i] = np.cross(z_i, T_end[0:3, 3] - p_i)
        J[3:6, i] = z_i

    return E, J


# ==========================================
# 2. 串行连续追踪求解 (保证分支一致性)
# ==========================================
if __name__ == "__main__":
    output_dir = "05"
    os.makedirs(output_dir, exist_ok=True)

    print("正在进行单核串行连续追踪，确保处于同一运动分支...")

    steps = 360
    q2_vals = np.linspace(0, 2 * np.pi, steps)

    # 初始微小扰动以防止完全卡在死点
    q_curr = np.random.normal(0, 0.05, 7)
    q_curr[1] = 0

    q_history = []
    sv_history = []
    valid_q2 = []

    for q2 in q2_vals:
        # 目标函数：寻找让闭环误差最小的被动关节角
        def obj(q_p):
            q_test = np.array([q_p[0], q2, q_p[1], q_p[2], q_p[3], q_p[4], q_p[5]])
            E, _ = get_model_data(q_test)
            return np.linalg.norm(E)


        # 核心：使用**上一帧**的角度作为初值，确保连续平滑移动
        q_p_guess = np.array([q_curr[0], q_curr[2], q_curr[3], q_curr[4], q_curr[5], q_curr[6]])
        res = minimize(obj, q_p_guess, method='BFGS')

        if res.fun < 1e-4:
            q_p = res.x
            q_curr = np.array([q_p[0], q2, q_p[1], q_p[2], q_p[3], q_p[4], q_p[5]])
            E, J = get_model_data(q_curr)

            # 提取 6x6 被动雅可比 (移除主动件 J2 的列)
            Jp = np.delete(J, 1, axis=1)
            _, S, _ = np.linalg.svd(Jp)

            q_history.append(q_curr.copy())
            sv_history.append(S)
            valid_q2.append(q2)
        else:
            print(f"在 J2 = {np.degrees(q2):.1f} 度时无法闭合，机构可能遭遇卡点跳出。")

    print(f"解算完成！成功获取 {len(valid_q2)} 组连续位形。")

    q_history = np.array(q_history)
    sv_history = np.array(sv_history)
    valid_q2 = np.array(valid_q2)

    # 数据平滑处理：消除超过 180 度的数值跳变
    for col in range(7):
        q_history[:, col] = np.unwrap(q_history[:, col])

    q2_deg = np.degrees(valid_q2)
    q_hist_deg = np.degrees(q_history)

    # ==========================================
    # 3. 数据导出为 CSV 表格
    # ==========================================
    print("📊 正在生成数据表格文件...")
    data_dict = {'J2_Drive_Angle_deg': q2_deg}
    for i in range(7):
        data_dict[f'J{i + 1}_Angle_deg'] = q_hist_deg[:, i]
    for i in range(6):
        data_dict[f'Singular_Value_{i + 1}'] = sv_history[:, i]

    df = pd.DataFrame(data_dict)
    csv_path = os.path.join(output_dir, "j2_kinematics_table.csv")
    df.to_csv(csv_path, index=False)
    print(f"📁 数据已成功保存至表格: {csv_path}")


    # ==========================================
    # 4. 傅里叶级数拟合代数公式
    # ==========================================
    def fourier_series(x, a0, a1, b1, a2, b2, a3, b3):
        return (a0 +
                a1 * np.cos(x) + b1 * np.sin(x) +
                a2 * np.cos(2 * x) + b2 * np.sin(2 * x) +
                a3 * np.cos(3 * x) + b3 * np.sin(3 * x))


    print("\n📈 ================== 傅里叶级数运动学公式 ==================")
    print("输入 x 为 J2 的角度(弧度rad)，输出被动关节 Qi 为角度(度deg)")

    labels = ['J1', 'J3', 'J4', 'J5', 'J6', 'J7']
    indices = [0, 2, 3, 4, 5, 6]

    for idx, label in zip(indices, labels):
        # 拟合：横坐标用弧度 (valid_q2)，纵坐标用角度 (q_hist_deg)
        popt, _ = curve_fit(fourier_series, valid_q2, q_hist_deg[:, idx])
        a0, a1, b1, a2, b2, a3, b3 = popt
        formula = (f"{label}(x) = {a0:+.3f} "
                   f"{a1:+.3f}*cos(x) {b1:+.3f}*sin(x) "
                   f"{a2:+.3f}*cos(2x) {b2:+.3f}*sin(2x) "
                   f"{a3:+.3f}*cos(3x) {b3:+.3f}*sin(3x)")
        print(formula)
    print("========================================================\n")

    # ==========================================
    # 5. 绘制综合分析图
    # ==========================================
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 12), sharex=True)

    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b']
    for idx, color, label in zip(indices, colors, labels):
        ax1.plot(q2_deg, q_hist_deg[:, idx], color=color, linewidth=2.5, label=f'Joint {label}')

    ax1.axhline(0, color='black', linewidth=1, linestyle='--')
    ax1.set_ylabel('Passive Joints Angles ($^\circ$)', fontsize=14, fontweight='bold')
    ax1.set_title('Continuous Kinematic Mapping & Singularity Analysis (Driven by J2)', fontsize=16, fontweight='bold')
    ax1.legend(loc='center left', bbox_to_anchor=(1, 0.5), fontsize=12)
    ax1.grid(True, linestyle=':', alpha=0.8)

    for i in range(6):
        if i == 5:
            ax2.plot(q2_deg, sv_history[:, i], 'r-', linewidth=3, label=f'Min SV ($\sigma_{i + 1}$)')
        elif i == 4:
            ax2.plot(q2_deg, sv_history[:, i], 'orange', linestyle='--', linewidth=2.5,
                     label=f'2nd Min SV ($\sigma_{i + 1}$)')
        else:
            ax2.plot(q2_deg, sv_history[:, i], linewidth=1.5, alpha=0.5, label=f'$\sigma_{i + 1}$')

    ax2.set_xlabel('Driving Joint J2 Angle ($^\circ$)', fontsize=14, fontweight='bold')
    ax2.set_ylabel('Singular Values of $J_p$', fontsize=14, fontweight='bold')
    ax2.legend(loc='center left', bbox_to_anchor=(1, 0.5), fontsize=12)
    ax2.grid(True, linestyle=':', alpha=0.8)

    if len(q2_deg) > 0:
        plt.xlim(0, max(q2_deg))

    plt.tight_layout()
    save_path = os.path.join(output_dir, "j2_combined_analysis.png")
    plt.savefig(save_path, dpi=300)
    print("可视化联合图表生成完毕！")
    plt.show()