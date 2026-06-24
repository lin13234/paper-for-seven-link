import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize, curve_fit
import os
import subprocess


# ==========================================
# 工具函数：将 Numpy 数组转化为 LaTeX bmatrix 格式
# ==========================================
def array_to_latex_bmatrix(arr, precision=3):
    """将 numpy array 转化为 LaTeX 的 bmatrix 字符串"""
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
# 2. 串行连续追踪求解 (以 q1 为主动关节)
# ==========================================
output_dir = "06_q10087"
os.makedirs(output_dir, exist_ok=True)

print("正在进行串行运动学追踪计算 (驱动关节: q1)...")
steps = 360
q1_vals = np.linspace(0, 2 * np.pi, steps)
q_curr = np.random.normal(0, 0.05, 7)
q_curr[0] = 0

q_history, sv_history, valid_q1 = [], [], []

for q1 in q1_vals:
    def obj(q_p):
        q_test = np.array([q1, q_p[0], q_p[1], q_p[2], q_p[3], q_p[4], q_p[5]])
        E, _ = get_model_data(q_test)
        return np.linalg.norm(E)


    q_p_guess = np.array([q_curr[1], q_curr[2], q_curr[3], q_curr[4], q_curr[5], q_curr[6]])
    res = minimize(obj, q_p_guess, method='BFGS')

    if res.fun < 1e-4:
        q_p = res.x
        q_curr = np.array([q1, q_p[0], q_p[1], q_p[2], q_p[3], q_p[4], q_p[5]])
        E, J = get_model_data(q_curr)
        Jp = np.delete(J, 0, axis=1)
        _, S, _ = np.linalg.svd(Jp)

        q_history.append(q_curr.copy())
        sv_history.append(S)
        valid_q1.append(q1)

q_history = np.array(q_history)
sv_history = np.array(sv_history)
valid_q1 = np.array(valid_q1)

for col in range(7):
    q_history[:, col] = np.unwrap(q_history[:, col])

q1_deg = np.degrees(valid_q1)
q_hist_deg = np.degrees(q_history)

# ==========================================
# 3. 提取关键帧矩阵
# ==========================================
q_example = q_history[0]
E_example, J_example = get_model_data(q_example)
Jp_example = np.delete(J_example, 0, axis=1)
U_example, S_example, Vt_example = np.linalg.svd(Jp_example)

latex_q = array_to_latex_bmatrix(q_example, 4)
latex_J = array_to_latex_bmatrix(J_example, 3)
latex_Jp = array_to_latex_bmatrix(Jp_example, 3)
latex_U = array_to_latex_bmatrix(U_example, 3)
latex_S = array_to_latex_bmatrix(np.diag(S_example), 3)
latex_Vt = array_to_latex_bmatrix(Vt_example, 3)

# ==========================================
# 4. 生成绘图并保存
# ==========================================
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 10), sharex=True)
colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b']
labels = ['J2', 'J3', 'J4', 'J5', 'J6', 'J7']
indices = [1, 2, 3, 4, 5, 6]

for idx, color, label in zip(indices, colors, labels):
    ax1.plot(q1_deg, q_hist_deg[:, idx], color=color, linewidth=2, label=f'Joint {label}')
ax1.set_ylabel('Passive Joints Angles (deg)', fontweight='bold')
ax1.set_title('Continuous Kinematic Mapping & SVD Analysis', fontweight='bold')
ax1.legend(loc='upper right')
ax1.grid(True, linestyle=':')

for i in range(6):
    if i == 5:
        ax2.plot(q1_deg, sv_history[:, i], 'r-', linewidth=2.5, label=f'Min SV ($\sigma_{i + 1}$)')
    else:
        ax2.plot(q1_deg, sv_history[:, i], linewidth=1.2, alpha=0.5)
ax2.set_xlabel('Driving Joint J1 Angle (deg)', fontweight='bold')
ax2.set_ylabel('Singular Values of $J_p$', fontweight='bold')
ax2.legend(loc='upper right')
ax2.grid(True, linestyle=':')
plt.xlim(0, 360)
img_path = "j1_analysis_plot.png"
plt.tight_layout()
plt.savefig(os.path.join(output_dir, img_path), dpi=300)
plt.close()

# ==========================================
# 5. 混合模型处理（J2-J6 采用 5阶傅里叶拟合，J7 采用 7阶多项式拟合）
# ==========================================
ORDER_FOURIER = 5
POLY_DEGREE = 7  # 设定 J7 关节为 7 阶多项式拟合
print(f"正在进行数据拟合（J7采用{POLY_DEGREE}阶多项式拟合，其他被动关节采用{ORDER_FOURIER}阶傅里叶拟合）...")


# 5阶傅里叶级数函数定义
def fourier_series_5th(x, a0, a1, b1, a2, b2, a3, b3, a4, b4, a5, b5):
    return (a0 +
            a1 * np.cos(x) + b1 * np.sin(x) +
            a2 * np.cos(2 * x) + b2 * np.sin(2 * x) +
            a3 * np.cos(3 * x) + b3 * np.sin(3 * x) +
            a4 * np.cos(4 * x) + b4 * np.sin(4 * x) +
            a5 * np.cos(5 * x) + b5 * np.sin(5 * x))


tex_code = r"""\documentclass[12pt,a4paper]{article}
\usepackage{amsmath}
\usepackage{graphicx}
\usepackage{geometry}
\usepackage{float}
\geometry{left=2cm,right=2cm,top=2.5cm,bottom=2.5cm}

\title{\textbf{Kinematic and Singularity Analysis Report \\ with Explicit Matrix Operations}}
\author{Automated Generation System}
\date{\today}

\begin{document}
\maketitle

\section{Mathematical Formulation}
The closed-loop spatial kinematic constraint is given by:
\begin{equation}
    T_1(q_1) T_2(q_2) T_3(q_3) T_4(q_4) T_5(q_5) T_6(q_6) T_7(q_7) = I
\end{equation}
The velocity constraint yields $J_p \dot{\mathbf{q}}_p = - J_a \dot{q}_1$, where $J_p$ is the $6 \times 6$ passive Jacobian matrix obtained by removing the column corresponding to the driving joint $J_1$.

\section{Example Matrix Calculation at $q_1 = 0^\circ$}
The solved full joint configuration (in radians) is:
\begin{equation}
    \mathbf{q} = """ + latex_q + r"""
\end{equation}

Based on this configuration, the full $6 \times 7$ Jacobian matrix $J$ is evaluated as:
\begin{equation}
    \resizebox{0.95\textwidth}{!}{$ J = """ + latex_J + r""" $}
\end{equation}

By extracting the columns corresponding to the passive joints, we form the passive Jacobian $J_p \in \mathbb{R}^{6 \times 6}$:
\begin{equation}
    \resizebox{0.85\textwidth}{!}{$ J_p = """ + latex_Jp + r""" $}
\end{equation}

Applying Singular Value Decomposition (SVD) on $J_p = U \Sigma V^T$, the decomposition yields $U$:
\begin{equation{
    \resizebox{0.85\textwidth}{!}{$ U = """ + latex_U + r""" $}
\end{equation}

The singular values matrix $\Sigma$ (diagonal):
\begin{equation}
    \resizebox{0.85\textwidth}{!}{$ \Sigma = """ + latex_S + r""" $}
\end{equation}

And the transposed right singular vectors matrix $V^T$:
\begin{equation}
    \resizebox{0.85\textwidth}{!}{$ V^T = """ + latex_Vt + r""" $}
\end{equation}

\section{Analysis Results \& Curve Fitting}
\begin{figure}[H]
    \centering
    \includegraphics[width=0.9\textwidth]{""" + img_path + r"""}
    \caption{Kinematic mapping and singular value evolution driven by $J_1$.}
\end{figure}

The empirical curve fitting formulas for the passive joints are derived as follows:
\begin{itemize}
"""

# 分别处理每个被动关节
for idx, label in zip(indices, labels):
    if label == 'J7':
        # ======= 关键修改点 1：对 J7 执行 7 阶多项式拟合 =======
        # 使用 np.polyfit 计算多项式系数，系数按从高次到低次的顺序排列 (p_7, p_6, ... p_0)
        poly_coefs = np.polyfit(valid_q1, q_hist_deg[:, idx], POLY_DEGREE)

        # 将多项式系数优雅地格式化拼接为标准的 LaTeX 数学公式
        poly_terms = []
        for power, coef in enumerate(poly_coefs):
            current_power = POLY_DEGREE - power
            if current_power > 1:
                poly_terms.append(f"{coef:+.3f}x^{{{current_power}}}")
            elif current_power == 1:
                poly_terms.append(f"{coef:+.3f}x")
            else:
                poly_terms.append(f"{coef:+.3f}")  # 常数项

        # 拼接各个子项组合出长多项式公式
        latex_poly_formula = " ".join(poly_terms)
        eq_str = (f"\\item \\textbf{{Joint {label} ({POLY_DEGREE}th-order Polynomial):}} \\\\ \n"
                  f"$Q_{{{label}}}(x) = {latex_poly_formula}$")
    else:
        # 非 J7 轴依旧采用标准的 5阶 傅里叶级数拟合
        popt, _ = curve_fit(fourier_series_5th, valid_q1, q_hist_deg[:, idx])
        a0, a1, b1, a2, b2, a3, b3, a4, b4, a5, b5 = popt
        eq_str = (f"\\item \\textbf{{Joint {label}:}} \\\\ "
                  f"$Q_{{{label}}}(x) = {a0:+.3f} {a1:+.3f}\\cos(x) {b1:+.3f}\\sin(x) "
                  f"{a2:+.3f}\\cos(2x) {b2:+.3f}\\sin(2x) {a3:+.3f}\\cos(3x) {b3:+.3f}\\sin(3x) "
                  f"{a4:+.3f}\\cos(4x) {b4:+.3f}\\sin(4x) {a5:+.3f}\\cos(5x) {b5:+.3f}\\sin(5x)$")

    tex_code += eq_str + "\n"

tex_code += r"""\end{itemize}
\end{document}
"""

tex_file_path = os.path.join(output_dir, "analysis_report_with_matrices.tex")
with open(tex_file_path, "w", encoding="utf-8") as f:
    f.write(tex_code)

print(f"✅ 包含多项式与傅里叶级数混合演算的 LaTeX 源码已生成: {tex_file_path}")

try:
    subprocess.run(["pdflatex", "-interaction=nonstopmode", "analysis_report_with_matrices.tex"],
                   cwd=output_dir, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    print("\n🎉 PDF 报告编译成功！")
except Exception as e:
    print("\n⚠️ 自动编译 PDF 失败（可能未安装 LaTeX）。请去文件夹手动编译 tex 文件。")