# symbolic_mdh_branch_solution

## 项目定位

这个目录保存 7R 闭链机构的符号消元、主分支后处理、数值验证、解析残差验证、对比结果与论文式推导文件。

当前策略：

1. 保留 `derive_mdh_symbolic.py` 的主符号消元结果。
2. 不重跑耗时主消元时，使用 `postprocess_branch_analysis.py` 做轻量后处理。
3. 如果没有可靠初等显式解，则 `implicit_solution.txt` 就是最终解析解文件。

## MDH 公式

采用修正 DH 形式：

```text
A_i = Rx(alpha_{i-1}) * Tx(a_{i-1}) * Rz(theta_i) * Tz(d_i)
```

对称关系：

```text
Delta theta1 = Delta theta7
Delta theta2 = -Delta theta6
Delta theta5 = -Delta theta3
```

定义：

```text
s = Delta theta3
x = Delta theta1
y = Delta theta2
z = Delta theta4
```

## 运行方式

### 1. 只做后处理，不重跑主符号消元

```powershell
python symbolic_mdh_branch_solution/postprocess_branch_analysis.py
```

这个脚本会读取已有：

- `equations_unique.txt`
- `independent_sets.txt`
- `elimination_results.txt`
- `main_branch_candidates.txt`
- `joint_solution_numeric.csv`

### 2. 主符号消元

只有在确认现有结果不足时才运行：

```powershell
python symbolic_mdh_branch_solution/derive_mdh_symbolic.py
```

### 3. 数值验证

```powershell
python symbolic_mdh_branch_solution/verify_numeric.py
```

它保留原有 361 点连续 `least_squares` 数值解功能；如果已经存在 `implicit_solution.txt` 或 `explicit_joint_functions.txt`，还会顺带刷新：

- `analytic_residual.csv`
- `analytic_residual.png`
- `comparison.png`
- `comparison_report.txt`

### 4. 运行测试

```powershell
pytest tests/test_symbolic_mdh_branch_solution.py -q
pytest tests/test_postprocess_branch_analysis.py -q
```

## 主要输出文件说明

### 主符号层

- `equations_unique.txt`：唯一闭环多项式。
- `independent_sets.txt`：`rank=3` 独立方程组。
- `elimination_results.txt`：已有 `Z->X` 主消元结果。
- `main_branch_candidates.txt`：已有主分支候选因子记录。

### 后处理层

- `check_report.txt`：对已有结果的审计报告。
- `elimination_path_report.txt`：补充多消元次序搜索记录。
- `refined_elimination_candidates.txt`：补充路径中成功得到的单主变量候选。
- `selected_main_branch.txt`：严格初始位形检查后的主分支选择结果。
- `Y_solution.txt`：关于 `Y(q)` 的说明；若无可靠初等显式解，会明确写成隐式分支说明。
- `Z_solution.txt`：`Z(Y,q)` 恢复关系。
- `X_solution.txt`：`X(Y,q)` 的二次关系或分支表达。
- `explicit_joint_functions.txt`：关节恢复关系汇总；如果最终仍为隐式解，这里会明确说明。
- `implicit_solution.txt`：最终隐式解析解文件；当无可靠显式解时，这个文件就是最终解析解。

### 验证与对比

- `joint_solution_numeric.csv`：原始数值连续求解结果。
- `joint_angles_vs_delta_theta3.png`：原始数值角度曲线。
- `analytic_residual.csv`：解析主分支代回完整 MDH 后的闭环残差采样。
- `analytic_residual.png`：解析残差图。
- `comparison.png`：解析解与数值解的角度误差图。
- `comparison_report.txt`：RMSE、最大误差、平均误差和解析残差统计。

### 推导文档

- `paper_derivation.md`：中文论文式推导说明。

## 最终解析解位置

如果存在可靠显式解，查看：

- `explicit_joint_functions.txt`

如果没有可靠显式解，查看：

- `implicit_solution.txt`

## 最终主分支位置

- `selected_main_branch.txt`

## 最终图像位置

- `joint_angles_vs_delta_theta3.png`
- `analytic_residual.png`
- `comparison.png`

## 最终论文推导位置

- `paper_derivation.md`
