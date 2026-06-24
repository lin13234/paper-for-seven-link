# 01Kinematics 数据整理与作图建议

## 1. 本次整理后的数据位置

已将 `D:\01STUDY\05yundongxue\DOF\01Kinematics` 中与论文主线最相关的数据整理到：

`E:\hub\02论文正文\001paper\05_推导与附录\01_原始计算数据`

其中分为两层：

- `01_可用核心数据`
  这一层是优先用于论文正文和作图的数据源。
- `02_待核查结果`
  这一层保留试算版本、异常拟合结果和多分支探索结果，当前不要直接引用到正文。

## 2. 核心数据与章节对应关系

### 第 2 章：机构构型与参数定义

对应目录：

- `01_可用核心数据\A_DH与机构构型`

对应文件：

- `01_DH.py`
- `Kinematics_Report.tex`
- `Kinematics_Report.pdf`
- `kinematics_MDH.png`
- `kinematics_final.png`

这一章建议放：

- 机构拓扑图
- 关节编号
- 对称平面
- D-H 参数表
- 初始构型示意图

### 第 3 章：自由度、奇异性与主分支分析

对应目录：

- `01_可用核心数据\B_q2主分支与奇异性`
- `01_可用核心数据\D_SVD与流形图`

对应文件：

- `05_q2.py`
- `j2_kinematics_table.csv`
- `j2_combined_analysis.png`
- `j2_analysis_plot.png`
- `Kinematics_Singularity_Report.tex`
- `group_theory_svd.png`
- `06svd.py`

这一章建议放：

- `q2` 驱动下的关节角变化曲线
- 被动雅可比最小奇异值曲线
- 奇异点位置表格
- 说明 `0/180/360 deg` 附近是真奇异，`90/270 deg` 附近是伪奇异或病态点

### 第 4 章：第二运动模式与退化平面分支

对应目录：

- `01_可用核心数据\C_q1退化分支`

对应文件：

- `kinematics_data.csv`
- `degenerated_report.tex`
- `degenerated_kinematics.png`
- `plot.png`
- `report.tex`

这一章建议放：

- `q1` 驱动下的第二运动模式示意
- 退化分支中固定关节与活动关节的区分
- 说明 `J2, J4, J6` 近似固定，`J3, J5, J7` 主导退化分支运动

注意：

- `degenerated_report.tex` 中的某些拟合系数仍然异常，特别是 `J7`，所以这章应优先使用原始轨迹和结构关系，不要直接引用里面的拟合公式。

### 第 5 章：流形与分支切换关系

对应目录：

- `01_可用核心数据\D_SVD与流形图`

对应文件：

- `Academic_Manifold_Clean.pdf`
- `Academic_Manifold_Clean_Fixed.pdf`
- `Academic_Manifold_Final.pdf`
- `Academic_Manifold_Final_Complete.pdf`
- `Academic_Manifold_Final_With_Axes.pdf`
- `Academic_Manifold_Intersections.pdf`
- `Robot_Manifold_Q1_Active.pdf`
- `manifold_3d.png`

这一章建议放：

- 两个运动分支的流形图
- 分支交点图
- 奇异位形作为切换入口的几何解释

## 3. 当前可直接使用的数据

### 可以直接作为正文数据源

1. `A_DH与机构构型` 整个目录
2. `B_q2主分支与奇异性` 整个目录
3. `C_q1退化分支\kinematics_data.csv`
4. `C_q1退化分支\degenerated_kinematics.png`
5. `D_SVD与流形图` 中的大部分 PDF/PNG

### 可以作为方法说明，但不建议直接展示太多

1. `06svd.py`
2. `16_jisuanhuizong.py`
3. `11_Ultimate_Report`

这些更适合支撑你自己写“计算流程”和“附录”，不适合整块放入正文。

## 4. 当前不建议直接用于正文的数据

对应目录：

- `02_待核查结果\q1试算版本`
- `02_待核查结果\q3多分支探索`

原因：

1. `q1` 试算版本中不同目录的结果不统一。
2. 有些 `J7` 的拟合系数明显发散，数值不可信。
3. `q3` 多分支探索结果数量太多，不符合当前论文“只有两种运动模式”的主线。

这部分的用途：

- 作为你自己回溯数值过程的备份。
- 后续如果需要补充“为什么最终只保留两种模式”，可以从这里找辅助说明。

## 5. 推荐主文图清单

### Figure 1

机构拓扑图、关节编号、对称平面、坐标系定义  
来源：`A_DH与机构构型`

### Figure 2

D-H 参数对应的初始机构构型图  
来源：`kinematics_MDH.png` 或 `kinematics_final.png`

### Figure 3

`q2` 驱动下各关节角随输入角变化的曲线  
来源：`j2_kinematics_table.csv`

建议内容：

- 画 `q1, q3, q4, q5, q6, q7` 对 `q2` 的曲线
- 不要把所有探索分支画进去

### Figure 4

`q2` 驱动下的被动雅可比奇异值曲线  
来源：`j2_combined_analysis.png`

建议内容：

- 高亮最小奇异值
- 标出 `0/180/360 deg`
- `90/270 deg` 可加说明为 pseudo-singular points

### Figure 5

主分支的对称约束关系图  
建议从 `j2_kinematics_table.csv` 重新作图

建议画法：

- `q7` vs `q1`
- `q6` vs `-q2`
- `q5` vs `-q3`

如果点落在对角线上，这张图会非常有说服力。

### Figure 6

`q1` 驱动下退化分支运动曲线  
来源：`kinematics_data.csv` 和 `degenerated_kinematics.png`

建议内容：

- 突出 `J2, J4, J6` 基本不动
- 突出 `J3, J5, J7` 才是主要运动变量

### Figure 7

两个运动分支的流形图  
来源：`Academic_Manifold_Final_With_Axes.pdf` 或 `manifold_3d.png`

### Figure 8

流形交点或分支切换示意图  
来源：`Academic_Manifold_Intersections.pdf`

这张图用来支撑“切换只发生在特定奇异位形附近”。

## 6. 推荐附录图或补充材料

以下内容适合放附录，不建议全进正文：

1. 完整雅可比矩阵示例
2. SVD 展开矩阵
3. 完整 Fourier 拟合系数
4. 多分支探索图
5. 试算失败或异常拟合版本

## 7. 当前最需要重新作的图

为了投稿 MMT，建议不要直接拿现有自动生成图就上，而是基于原始 CSV 重新统一绘图风格。

优先重画：

1. `q2` 主分支关节角曲线
2. `q2` 最小奇异值曲线
3. 主分支约束关系散点图
4. `q1` 退化分支关节角曲线
5. 分支切换流形图

统一绘图要求：

- 横轴单位统一为 `deg`
- 纵轴单位统一为 `deg` 或 singular value
- 图例使用 `q_1, q_2, ...` 或 `J1, J2, ...`，不要混用
- 字体、线宽、颜色和图例位置统一
- 图题不要写成长句，说明放在 caption 里

## 8. 当前不建议直接写进正文的公式

先不要直接引用这些拟合表达：

1. 某些 `q1` 分支下的 `J7` Fourier 拟合
2. `degenerated_report.tex` 中异常巨大的 `J7` 系数
3. 所有来源不一致的多版本 `q1` 试算公式

原因：

- 这些公式大概率受到了角度展开、分支跳跃或拟合模型不合适的影响。
- 先用原始轨迹证明“存在第二分支”，比强行给出错误解析式更稳。

## 9. 建议的下一步

现在已经可以正式进入做图阶段。

推荐顺序：

1. 先基于 `j2_kinematics_table.csv` 重画第 3、4、5 张图。
2. 再基于 `kinematics_data.csv` 重画退化分支图。
3. 再从流形 PDF/PNG 中筛最终留在正文的一版。
4. 最后统一所有图的论文风格。

## 10. 一句话原则

这篇论文当前只讲两种运动模式，所以所有图和数据都要服务这条主线：

`q2` 主空间分支 + `q1` 退化平面分支 + 奇异位形切换关系

凡是不能直接增强这条主线的试算结果，先放到待核查区，不进正文。

