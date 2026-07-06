# symbolic_mdh_joint2

本目录只做 Joint2 输入情形下的 Modified DH 闭环方程建立。

固定约束：
- 关节编号始终保持 Joint1~Joint7。
- 输入变量为 s = Δθ2。
- 未知量为 x = Δθ1, y = Δθ3, z = Δθ4。
- 角度表达式固定为 θ1=x, θ2=s, θ3=π+y, θ4=z, θ5=-y, θ6=π-s, θ7=x。
- 只输出 MDH、闭环、k=4 分块和半角代换结果。
- 禁止消元、Groebner、Resultant、RootOf、Branch Analysis、Fourier、SVD、数值拟合。

文件说明：
- mdh_matrices.txt：A1~A7 的 Latex / SymPy / 代码表示。
- closure_matrix.txt：完整闭环矩阵 T 及零位误差。
- block_equations.txt：L=A1A2A3A4 与 R=(A5A6A7)^(-1) 的 16 个元素方程。
- half_angle_equations.txt：16 个元素方程的半角代换、分子分母、factor 和总次数。
- equation_summary.txt：16 个方程的变量出现情况与次数总览。
- derive_joint2_mdh.py：本阶段唯一推导脚本。

运行方式：
```bash
python symbolic_mdh_joint2/derive_joint2_mdh.py
```