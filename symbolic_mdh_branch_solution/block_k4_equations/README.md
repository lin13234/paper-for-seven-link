# block_k4_equations

这个目录只保存 `k=4` 分块闭环方程的显式列式结果，不包含任何自动求解或自动消元。

## 文件说明

- `block_k4_left_matrix.txt`
  保存左侧矩阵 `L = A1*A2*A3*A4` 的完整 4x4 元素表达式。
- `block_k4_right_matrix.txt`
  保存右侧矩阵 `R = inv(A7)*inv(A6)*inv(A5)` 的完整 4x4 元素表达式。
- `block_k4_equations_trig.txt`
  保存 16 个矩阵元素闭环方程 `Lij - Rij = 0` 的三角形式。
- `block_k4_equations_half_angle.txt`
  保存同一批 16 个方程在半角代换后的表达式、分子分母、因式分解和复杂度统计。
- `block_k4_equation_summary.csv`
  保存 16 个方程的统计表，并按复杂度评分排序。
- `block_k4_equations_latex.tex`
  保存适合直接复制到论文中的 LaTeX 版本。

## 哪个文件对应三角形式

- `block_k4_equations_trig.txt`

## 哪个文件对应半角形式

- `block_k4_equations_half_angle.txt`

## 哪个文件适合后续人工消元

- `block_k4_equations_trig.txt`
- `block_k4_equations_half_angle.txt`
- `block_k4_equation_summary.csv`

其中：
- 三角形式适合先做人工结构观察；
- 半角形式适合人工挑选低复杂度方程；
- CSV 适合先按复杂度和变量出现情况快速筛读。

## 后续工作原则

后续不再由脚本自动求解，而是由人工从这 16 个显式方程中选择合适的方程，开展解析推导。
