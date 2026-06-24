import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import simpson
import os
import subprocess


def solve_and_generate_16_folder(a0=40.0, a1=80.0, d2=100.0, a3=100.0):
    print(f"🚀 启动 7R 机构终极解析引擎 | 参数: a0={a0}, a1={a1}, d2={d2}, a3={a3}\n")

    # 1. 文件夹初始化
    output_dir = "16"
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # 2. 纯代数显式数据生成 (基于闭环几何推导)
    # 位置方程: A*cos(q2)*cos(q3) + B*sin(q3) + C = 0
    A = 2 * a1 * a3
    B = 2 * a3 * d2
    C = a0 ** 2 + a1 ** 2 + d2 ** 2 - a3 ** 2

    steps = 360
    q2_vals = np.linspace(0, 2 * np.pi, steps, endpoint=False)
    q_data = np.zeros((steps, 7))

    print("⏳ 正在根据解析显式解计算轨迹...")
    for i, q2 in enumerate(q2_vals):
        # 求解 tan(q3/2) 的二次方程: (C - A*cos(q2))t^2 + 2Bt + (C + A*cos(q2)) = 0
        a_q = C - A * np.cos(q2)
        b_q = 2 * B
        c_q = C + A * np.cos(q2)
        delta = b_q ** 2 - 4 * a_q * c_q
        t_val = (-b_q + np.sqrt(max(0, delta))) / (2 * a_q)

        q3 = 2 * np.arctan(t_val)
        q4 = -2 * np.arctan(np.cos(q3) * np.tan(q2))
        q1 = np.arctan2(np.sin(q3) * np.cos(q2), np.cos(q3))

        # 记录 7 个关节 (包含镜像跟随关系)
        q_data[i] = [q1, q2, q3, q4, -q3, -q2, q1]

    for j in range(7):
        q_data[:, j] = np.unwrap(q_data[:, j])

    # 3. 提取傅里叶级数 (真实计算系数)
    def get_f_str(x, y):
        a0_f = (1 / (2 * np.pi)) * simpson(y=y, x=x)
        s = f"{np.degrees(a0_f):.4f}"
        for n in [1, 2, 3]:
            an = (1 / np.pi) * simpson(y=y * np.cos(n * x), x=x)
            bn = (1 / np.pi) * simpson(y=y * np.sin(n * x), x=x)
            if abs(np.degrees(an)) > 0.01:
                s += f" {'+' if an > 0 else '-'} {abs(np.degrees(an)):.4f} cos({n} θ2)"
            if abs(np.degrees(bn)) > 0.01:
                s += f" {'+' if bn > 0 else '-'} {abs(np.degrees(bn)):.4f} sin({n} θ2)"
        return s

    # 4. 预处理所有字符串 (彻底避开 f-string 反斜杠坑)
    f1_raw = get_f_str(q2_vals, q_data[:, 0])
    f3_raw = get_f_str(q2_vals, q_data[:, 2])
    f4_raw = get_f_str(q2_vals, q_data[:, 3])

    # 准备 LaTeX 专用的字符串 (在外部处理好反斜杠)
    f1_tex = f1_raw.replace('θ2', r'\theta_2').replace('cos', r'\cos').replace('sin', r'\sin')
    f3_tex = f3_raw.replace('θ2', r'\theta_2').replace('cos', r'\cos').replace('sin', r'\sin')
    f4_tex = f4_raw.replace('θ2', r'\theta_2').replace('cos', r'\cos').replace('sin', r'\sin')

    # 符号推导结果带入常数
    t_expr_latex = rf"t(\theta_2) = \frac{{ -{B:.0f} + \sqrt{{ {B ** 2 - C ** 2:.0f} + {A ** 2:.0f}\cos^2(\theta_2) }} }}{{ {C:.0f} - {A:.0f}\cos(\theta_2) }}"

    # 5. 控制台打印
    print("\n" + "=" * 60)
    print("🎯 符号推导结果 (带入参数化简后):")
    print(
        f"t(theta2) = (-{B:.0f} + sqrt({B ** 2 - C ** 2:.0f} + {A ** 2:.0f}*cos^2(theta2))) / ({C:.0f} - {A:.0f}*cos(theta2))")
    print("\n📊 傅里叶级数分解结果 (单位: 度):")
    print(f"theta1 = {f1_raw}")
    print(f"theta3 = {f3_raw}")
    print(f"theta4 = {f4_raw}")
    print("=" * 60 + "\n")

    # 6. 绘图 (绘制除主动角外所有关节)
    plt.figure(figsize=(10, 6))
    joints_to_plot = ['theta1', 'theta3', 'theta4', 'theta5', 'theta6', 'theta7']
    indices = [0, 2, 3, 4, 5, 6]
    for idx, name in zip(indices, joints_to_plot):
        plt.plot(np.degrees(q2_vals), np.degrees(q_data[:, idx]), label=name, lw=2)
    plt.title("Joint Trajectories"), plt.xlabel("theta2 (deg)"), plt.ylabel("Joint Angles (deg)")
    plt.legend(), plt.grid(True)
    plt.savefig(os.path.join(output_dir, "joint_plot.png"), dpi=300)
    plt.close()

    # 7. 生成 LaTeX 报告 (使用 .format 避开 f-string 语法限制)
    latex_tmpl = r"""
\documentclass[12pt]{article}
\usepackage[UTF8]{ctex}
\usepackage{amsmath, graphicx, geometry}
\geometry{left=2cm,right=2cm,top=2cm,bottom=2cm}
\begin{document}
\title{7R 机构全维度符号推导与数值分析报告}
\author{自动分析系统} \maketitle

\section{1. 符号推导与解析显式解}
代入参数 $a_0={a0}, a_1={a1}, d_2={d2}, a_3={a3}$。
首先计算中间变量 $t = \tan(\theta_3/2)$ 的显式代数式：
\begin{equation} {t_formula} \end{equation}

解析显式映射关系：
\begin{align}
\theta_3 &= 2 \arctan(t) \\
\theta_4 &= -2 \arctan(\cos(\theta_3) \tan(\theta_2)) \\
\theta_1 &= \text{arctan2}(\sin(\theta_3)\cos(\theta_2), \cos(\theta_3))
\end{align}

\section{2. 傅里叶级数分解 (程序计算结果)}
基于解析轨迹积分提取的数值显式解（单位：度）：
\begin{align}
\theta_1 &\approx {f1} \\
\theta_3 &\approx {f3} \\
\theta_4 &\approx {f4}
\end{align}

\section{3. 随动关节轨迹图}
\begin{figure}[h] \centering \includegraphics[width=0.85\textwidth]{joint_plot.png} \end{figure}
\end{document}
"""
    # 彻底杜绝 f-string 报错：使用传统的 format 方法
    tex_content = latex_tmpl.format(
        a0=a0, a1=a1, d2=d2, a3=a3,
        t_formula=t_expr_latex,
        f1=f1_tex, f3=f3_tex, f4=f4_tex
    )

    tex_file = os.path.join(output_dir, "Report.tex")
    with open(tex_file, "w", encoding="utf-8") as f:
        f.write(tex_content)

    print("⏳ 正在编译 PDF (请确保系统中装有 xelatex)...")
    try:
        subprocess.run(["xelatex", "-interaction=nonstopmode", "Report.tex"], cwd=output_dir, stdout=subprocess.PIPE,
                       stderr=subprocess.PIPE)
        print(f"🎉 任务完成！结果保存在文件夹 16 中。")
    except:
        print("未检测到 xelatex 环境，请在 16 文件夹手动编译 Report.tex。")


if __name__ == "__main__":
    solve_and_generate_16_folder(40, 80, 100, 100)