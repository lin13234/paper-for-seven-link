import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize
import os
import subprocess


# ==========================================
# 工具函数：将 Numpy 数组转化为 LaTeX bmatrix 格式
# ==========================================
def array_to_latex_bmatrix(arr, precision=3):
    if len(arr.shape) == 1:
        lines = [f"{val:.{precision}f}" for val in arr]
        return "\\begin{bmatrix} " + " \\\\ ".join(lines) + " \\end{bmatrix}"
    elif len(arr.shape) == 2:
        lines = []
        for row in arr:
            lines.append(" & ".join([f"{val:.{precision}f}" for val in row]))
        return "\\begin{bmatrix}\n  " + " \\\\\n  ".join(lines) + "\n\\end{bmatrix}"
    return ""


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


def get_model_data(q, l=1.0, a=2.0, b=2.5):
    params = [
        (0, l, 0, 0),
        (np.pi / 2, 2 * l, 2.5 * l, 0),
        (np.pi / 2, 0, 0, np.pi),
        (np.pi / 2, 2.5 * l, 0, 0),
        (np.pi / 2, 2.5 * l, 0, 0),
        (np.pi / 2, 0, 2.5 * l, np.pi),
        (np.pi / 2, 2 * l, 0, 0)
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
# 2. 多分支探测与连续追踪求解 (以 q1 为主动关节)
# ==========================================
output_dir = "06_q1_1"
os.makedirs(output_dir, exist_ok=True)

print("正在进行全空间多分支探测计算 (驱动关节: q1)...")
steps = 360
q1_vals = np.linspace(0, 2 * np.pi, steps)

# 【核心修改】：提供极为丰富的初始猜测值，涵盖各种折叠和翻转状态
initial_guesses = [
    np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1]),
    np.array([-0.1, -0.1, -0.1, -0.1, -0.1, -0.1]),
    np.array([np.pi / 2, -np.pi / 2, np.pi / 2, -np.pi / 2, np.pi / 2, -np.pi / 2]),
    np.array([-np.pi / 2, np.pi / 2, -np.pi / 2, np.pi / 2, -np.pi / 2, np.pi / 2]),
    np.array([np.pi, np.pi, np.pi, np.pi, np.pi, np.pi]),
    np.array([np.pi / 2, np.pi / 2, np.pi / 2, np.pi / 2, np.pi / 2, np.pi / 2]),
    np.array([-np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2]),
    np.array([np.pi, 0.1, np.pi, 0.1, np.pi, 0.1]),
    np.array([0.1, np.pi, 0.1, np.pi, 0.1, np.pi])
]

branches_data = []

for guess_idx, q_guess_start in enumerate(initial_guesses):
    # 【核心修改】：移除了 >=3 的限制，或者你也可以设为 >=4
    if len(branches_data) >= 4:
        break


    def obj_start(q_p):
        q_test = np.array([0.0, q_p[0], q_p[1], q_p[2], q_p[3], q_p[4], q_p[5]])
        E, _ = get_model_data(q_test)
        return np.linalg.norm(E)


    res_start = minimize(obj_start, q_guess_start, method='BFGS')
    if res_start.fun > 1e-4:
        continue

    start_q_p = res_start.x
    start_q_p = np.mod(start_q_p + np.pi, 2 * np.pi) - np.pi

    is_duplicate = False
    for branch in branches_data:
        existing_start = branch['q_history'][0, 1:]
        existing_start = np.mod(existing_start + np.pi, 2 * np.pi) - np.pi
        # 去重判定阈值设定为 0.1 弧度
        if np.linalg.norm(start_q_p - existing_start) < 0.1:
            is_duplicate = True
            break

    if is_duplicate:
        continue

    print(f"🌟 发现新分支 (Branch {chr(65 + len(branches_data))})！开始连续追踪...")
    q_history, sv_history, valid_q1 = [], [], []
    q_p_guess = res_start.x

    for q1 in q1_vals:
        def obj(q_p):
            q_test = np.array([q1, q_p[0], q_p[1], q_p[2], q_p[3], q_p[4], q_p[5]])
            E, _ = get_model_data(q_test)
            return np.linalg.norm(E)


        res = minimize(obj, q_p_guess, method='BFGS')

        if res.fun < 1e-4:
            q_p = res.x
            q_p_guess = q_p
            q_curr = np.array([q1, q_p[0], q_p[1], q_p[2], q_p[3], q_p[4], q_p[5]])
            E, J = get_model_data(q_curr)
            Jp = np.delete(J, 0, axis=1)  # 剔除 J1
            _, S, _ = np.linalg.svd(Jp)

            q_history.append(q_curr.copy())
            sv_history.append(S)
            valid_q1.append(q1)
        else:
            print(f"  ⚠️ 分支在 q1 = {np.degrees(q1):.1f}° 处断裂 (遇到奇异/极限)，停止该分支追踪。")
            break

    q_history = np.array(q_history)
    for col in range(7):
        q_history[:, col] = np.unwrap(q_history[:, col])

    branches_data.append({
        'name': chr(65 + len(branches_data)),  # 自动命名 A, B, C, D...
        'q_history': q_history,
        'sv_history': np.array(sv_history),
        'valid_q1': np.array(valid_q1)
    })

print(f"✅ 探测完毕！共提取到 {len(branches_data)} 个独立的运动分支。")

# ==========================================
# 3. 分别为每一个分支单独绘图并保存
# ==========================================
colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b']
labels = ['J2', 'J3', 'J4', 'J5', 'J6', 'J7']
indices = [1, 2, 3, 4, 5, 6]
saved_image_paths = []

for branch in branches_data:
    b_name = branch['name']
    q_hist_deg = np.degrees(branch['q_history'])
    q1_deg = np.degrees(branch['valid_q1'])
    sv_hist = branch['sv_history']

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 10), sharex=True)

    # 子图1：运动学映射
    for idx, color, label in zip(indices, colors, labels):
        ax1.plot(q1_deg, q_hist_deg[:, idx], color=color, linewidth=2, label=f'Joint {label}')

    ax1.set_ylabel('Passive Joints Angles (deg)', fontweight='bold')
    ax1.set_title(f'Kinematic Mapping & SVD - Branch {b_name} (Driven by J1)', fontweight='bold')
    ax1.legend(loc='upper right', ncol=2)
    ax1.grid(True, linestyle=':')

    # 子图2：奇异值演化
    for i in range(6):
        if i == 5:
            ax2.plot(q1_deg, sv_hist[:, i], 'r-', linewidth=2.5, label=f'Min SV ($\sigma_{i + 1}$)')
        else:
            ax2.plot(q1_deg, sv_hist[:, i], linewidth=1.5, alpha=0.6)

    ax2.set_xlabel('Driving Joint J1 Angle (deg)', fontweight='bold')
    ax2.set_ylabel('Singular Values of $J_p$', fontweight='bold')
    ax2.legend(loc='upper right')
    ax2.grid(True, linestyle=':')

    if len(q1_deg) > 0:
        plt.xlim(0, max(q1_deg))

    img_name = f"j1_branch_{b_name}_plot.png"
    img_path = os.path.join(output_dir, img_name)
    plt.tight_layout()
    plt.savefig(img_path, dpi=300)
    plt.close(fig)

    saved_image_paths.append(img_name)
    print(f"🎉 Branch {b_name} 的独立曲线图已保存至: {img_path}")

# ==========================================
# 4. 生成 LaTeX 报告 (包含所有分支的独立图像)
# ==========================================
if len(branches_data) > 0:
    print("📝 正在生成包含所有分支图像的 LaTeX 报告...")

    branchA = branches_data[0]
    q_example = branchA['q_history'][0]
    E_example, J_example = get_model_data(q_example)
    Jp_example = np.delete(J_example, 0, axis=1)
    U_example, S_example, Vt_example = np.linalg.svd(Jp_example)

    latex_q = array_to_latex_bmatrix(q_example, 4)
    latex_J = array_to_latex_bmatrix(J_example, 3)
    latex_Jp = array_to_latex_bmatrix(Jp_example, 3)
    latex_U = array_to_latex_bmatrix(U_example, 3)
    latex_S = array_to_latex_bmatrix(np.diag(S_example), 3)
    latex_Vt = array_to_latex_bmatrix(Vt_example, 3)

    tex_code = r"""\documentclass[12pt,a4paper]{article}
\usepackage{amsmath}
\usepackage{graphicx}
\usepackage{geometry}
\usepackage{float}
\geometry{left=2cm,right=2cm,top=2.5cm,bottom=2.5cm}

\title{\textbf{Multi-Branch Kinematic Analysis Report \\ (Separate Branch Visualization)}}
\author{Automated Generation System}
\date{\today}

\begin{document}
\maketitle

\section{Assembly Modes (Branches) Overview}
Due to the highly nonlinear nature of the closed-loop constraints, spatial 7R mechanisms typically exhibit multiple assembly modes (kinematic branches) for the same driving input. Our algorithm successfully identified \textbf{""" + str(
        len(branches_data)) + r"""} distinct branches by applying diverse initial perturbations at $q_1 = 0^\circ$.

\section{Matrix Calculation (Branch A Example at $q_1 = 0^\circ$)}
We extract the matrices for the first identified branch (Branch A). The full joint configuration is:
\begin{equation}
    \mathbf{q}_A = """ + latex_q + r"""
\end{equation}

The passive Jacobian $J_p \in \mathbb{R}^{6 \times 6}$ (removing column 1):
\begin{equation}
    \resizebox{0.85\textwidth}{!}{$ J_p = """ + latex_Jp + r""" $}
\end{equation}

Singular Values matrix $\Sigma$:
\begin{equation}
    \resizebox{0.85\textwidth}{!}{$ \Sigma = """ + latex_S + r""" $}
\end{equation}

\clearpage
\section{Kinematic \& Singularity Plots for Each Branch}
The continuous kinematic mappings and corresponding Singular Value Decomposition (SVD) evolutions for each identified branch are plotted below.
"""

    for b_idx, img_name in enumerate(saved_image_paths):
        b_name = branches_data[b_idx]['name']
        tex_code += r"""
\begin{figure}[H]
    \centering
    \includegraphics[width=0.9\textwidth]{""" + img_name + r"""}
    \caption{Kinematic mapping and singular value evolution for \textbf{Branch """ + b_name + r"""} (Driven by $J_1$).}
\end{figure}
"""

    tex_code += r"\end{document}"

    tex_file_path = os.path.join(output_dir, "j1_separate_branches_report.tex")
    with open(tex_file_path, "w", encoding="utf-8") as f:
        f.write(tex_code)

    try:
        subprocess.run(["pdflatex", "-interaction=nonstopmode", "j1_separate_branches_report.tex"],
                       cwd=output_dir, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        print("🎉 PDF 报告编译成功！请去 06_q1 文件夹查看完整报告。")
    except Exception as e:
        print("⚠️ PDF 编译失败。请前往文件夹手动编译。")