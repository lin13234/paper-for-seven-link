import os
import numpy as np
import matplotlib.pyplot as plt
import subprocess
from mpl_toolkits.mplot3d import Axes3D

# ==========================================
# 1. 配置与参数
# ==========================================
OUTPUT_DIR = "01DHresult"
if not os.path.exists(OUTPUT_DIR):
    os.makedirs(OUTPUT_DIR)

# MDH 参数: [alpha_{i-1}, a_{i-1}, d_i, theta_i_offset]
dh_params = [
    [0.0, 40.0, 0.0, 0.0],  # Joint 1
    [90.0, 80.0, 100.0, 0.0],  # Joint 2
    [90.0, 0.0, 0.0, 180.0],  # Joint 3
    [90.0, 100.0, 0.0, 0.0],  # Joint 4
    [90.0, 100.0, 0.0, 0.0],  # Joint 5
    [90.0, 0.0, 100.0, 180.0],  # Joint 6
    [90.0, 80.0, 0.0, 0.0]  # Joint 7
]


# ==========================================
# 2. 核心数学函数
# ==========================================
def to_actual_world(vec):
    """将 DH 坐标映射为实际全局坐标 (X, Z, Y) 以实现 Y 向上"""
    vec = np.asarray(vec)
    if vec.ndim == 1:
        return np.array([vec[0], vec[2], vec[1]])
    else:
        res = np.copy(vec)
        res[:, 1] = vec[:, 2]
        res[:, 2] = vec[:, 1]
        return res


def get_dh_matrix_MDH(alpha_deg, a, d, offset_deg):
    """改进型 DH 矩阵计算 (Modified DH)"""
    theta = np.radians(offset_deg)
    alpha = np.radians(alpha_deg)
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)

    return np.array([
        [ct, -st, 0, a],
        [st * ca, ct * ca, -sa, -d * sa],
        [st * sa, ct * sa, ca, d * ca],
        [0, 0, 0, 1]
    ])


def draw_rotation_arrow(ax, origin_dh, z_axis_dh, x_axis_dh, y_axis_dh, radius=18, color='orange', label=''):
    """
    在 3D 空间绘制旋转箭头。
    优化了箭头的比例，使其在视觉上更加协调。
    """
    x_dir = x_axis_dh / np.linalg.norm(x_axis_dh)
    y_dir = y_axis_dh / np.linalg.norm(y_axis_dh)

    # 生成 270 度的弧线
    angles = np.linspace(0, 1.5 * np.pi, 30)
    # 将圆盘稍微沿 Z 轴抬高一点点，避免与坐标轴重合
    center = origin_dh + z_axis_dh * 2.0

    arc_points = np.array([center + radius * np.cos(a) * x_dir + radius * np.sin(a) * y_dir for a in angles])
    arc_world = to_actual_world(arc_points)

    # 绘制弧线
    ax.plot(arc_world[:, 0], arc_world[:, 1], arc_world[:, 2], color=color, linewidth=1.2, alpha=0.8)

    # 绘制末端箭头：计算末端切向
    end_angle = angles[-1]
    tangent_dh = -np.sin(end_angle) * x_dir + np.cos(end_angle) * y_dir

    # 转换切向矢量并绘图，设置 arrow_length_ratio 使得箭头大小适中
    end_pt_world = arc_world[-1]
    tangent_world = to_actual_world(tangent_dh)

    ax.quiver(end_pt_world[0], end_pt_world[1], end_pt_world[2],
              tangent_world[0], tangent_world[1], tangent_world[2],
              color=color, length=radius * 0.6, arrow_length_ratio=0.4, linewidth=1.5)

    # 标注文字
    text_pos = to_actual_world(center + radius * 1.2 * x_dir)
    ax.text(*text_pos, label, color=color, fontsize=9, fontweight='bold')


# ==========================================
# 3. 报告生成逻辑 (LaTeX)
# ==========================================
def generate_pdf_report(dh_params, matrices, error_dist, T_final, img_name):
    tex_path = os.path.join(OUTPUT_DIR, "Report.tex")

    dh_table = ""
    for i, p in enumerate(dh_params):
        dh_table += f"{i + 1} & {p[0]} & {p[1]} & {p[2]} & {p[3]} \\\\ \\hline\n"

    coord_table = ""
    for i, T in enumerate(matrices):
        pos = to_actual_world(T[:3, 3])
        name = f"Frame {i}" if i > 0 else "Base"
        coord_table += f"{name} & {pos[0]:.2f} & {pos[1]:.2f} & {pos[2]:.2f} \\\\ \\hline\n"

    matrix_str = ""
    for row in T_final:
        matrix_str += " & ".join([f"{v:.3f}" for v in row]) + " \\\\ \n"

    latex_content = rf"""
\documentclass{{article}}
\usepackage[utf8]{{inputenc}}
\usepackage{{graphicx}}
\usepackage{{amsmath}}
\usepackage{{geometry}}
\usepackage{{booktabs}}
\geometry{{a4paper, margin=0.8in}}

\title{{Robot Kinematics MDH Verification Report}}
\author{{Coding Assistant}}
\date{{\today}}

\begin{{document}}
\maketitle

\section{{DH Parameters}}
\begin{{center}}
\begin{{tabular}}{{|c|c|c|c|c|}}
\hline
Joint & $\alpha_{{i-1}}$ (deg) & $a_{{i-1}}$ (mm) & $d_i$ (mm) & $\theta_{{off}}$ (deg) \\ \hline
{dh_table}
\end{{tabular}}
\end{{center}}

\section{{Visualization Result}}
\begin{{figure}}[h!]
\centering
\includegraphics[width=0.9\textwidth]{{{img_name}}}
\caption{{Kinematic Chain Visualization}}
\end{{figure}}

\section{{Diagnostic Data}}
\begin{{itemize}}
    \item \textbf{{Closure Error:}} {error_dist:.4f} mm
    \item \textbf{{Status:}} {"Perfectly Closed" if error_dist < 1e-3 else "Pose Misaligned"}
\end{{itemize}}

\subsection{{Final Transform Matrix ($T_{{base}}^{{7}}$)}}
\[
T = \begin{{bmatrix}}
{matrix_str}
\end{{bmatrix}}
\]

\subsection{{Joint Origin Coordinates (Actual World)}}
\begin{{center}}
\begin{{tabular}}{{|c|c|c|c|}}
\hline
Frame & X & Y (Up) & Z \\ \hline
{coord_table}
\end{{tabular}}
\end{{center}}

\end{{document}}
"""
    with open(tex_path, "w", encoding="utf-8") as f:
        f.write(latex_content)

    print("--- 正在生成 PDF (pdflatex) ---")
    try:
        subprocess.run(["pdflatex", "-interaction=nonstopmode", "Report.tex"],
                       cwd=OUTPUT_DIR, stdout=subprocess.DEVNULL)
        print(f"✅ 报告已保存在: {os.path.abspath(OUTPUT_DIR)}")
    except Exception as e:
        print(f"❌ PDF 编译失败。错误: {e}")


# ==========================================
# 4. 主流程
# ==========================================
def main():
    matrices = [np.eye(4)]
    T_current = np.eye(4)

    for params in dh_params:
        T_local = get_dh_matrix_MDH(*params)
        T_current = T_current @ T_local
        matrices.append(T_current)

    pos_dh = np.array([T[:3, 3] for T in matrices])
    pos_actual = to_actual_world(pos_dh)
    error_distance = np.linalg.norm(pos_actual[-1] - pos_actual[0])

    # 绘图设置
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(pos_actual[:, 0], pos_actual[:, 1], pos_actual[:, 2], '--ko', linewidth=2, alpha=0.5, label='Chain')

    # 设置等比例缩放，防止变形
    ax.set_box_aspect([1, 1, 1])

    axis_len = 30  # 坐标轴长度
    for i, T in enumerate(matrices):
        orig = to_actual_world(T[:3, 3])

        # 提取各个轴的方向并映射
        dir_x = to_actual_world(T[:3, 0])
        dir_y = to_actual_world(T[:3, 1])
        dir_z = to_actual_world(T[:3, 2])

        # 绘制坐标轴：通过 length 控制长度，arrow_length_ratio 控制箭头比例
        ax.quiver(orig[0], orig[1], orig[2], dir_x[0], dir_x[1], dir_x[2],
                  color='r', length=axis_len, arrow_length_ratio=0.2, alpha=0.7)
        ax.quiver(orig[0], orig[1], orig[2], dir_y[0], dir_y[1], dir_y[2],
                  color='g', length=axis_len, arrow_length_ratio=0.2, alpha=0.7)
        ax.quiver(orig[0], orig[1], orig[2], dir_z[0], dir_z[1], dir_z[2],
                  color='b', length=axis_len, arrow_length_ratio=0.3, linewidth=2)

        ax.text(orig[0], orig[1], orig[2], f"J{i}", fontsize=10, fontweight='bold')

        # 在 MDH 中，关节 i 的旋转绕 Z_i 进行
        if i > 0:
            draw_rotation_arrow(ax, T[:3, 3], T[:3, 2], T[:3, 0], T[:3, 1], label=f"$\\theta_{i}$")

    ax.set_xlabel('X (Actual)')
    ax.set_ylabel('Y (Actual - Up)')
    ax.set_zlabel('Z (Actual)')
    ax.set_title(f"Robot MDH Verification\nError: {error_distance:.4f} mm")

    # 自动调整视角范围
    all_pos = pos_actual
    max_range = np.ptp(all_pos, axis=0).max() / 2.0
    mid = (all_pos.max(axis=0) + all_pos.min(axis=0)) / 2.0
    ax.set_xlim(mid[0] - max_range, mid[0] + max_range)
    ax.set_ylim(mid[1] - max_range, mid[1] + max_range)
    ax.set_zlim(mid[2] - max_range, mid[2] + max_range)

    ax.view_init(elev=-50, azim=70,roll=20)
    img_filename = "kinematics_final.png"
    plt.savefig(os.path.join(OUTPUT_DIR, img_filename), dpi=300, bbox_inches='tight')

    # 执行 PDF 报告生成
    generate_pdf_report(dh_params, matrices, error_distance, np.round(matrices[-1], 3), img_filename)

    plt.show()


if __name__ == "__main__":
    main()