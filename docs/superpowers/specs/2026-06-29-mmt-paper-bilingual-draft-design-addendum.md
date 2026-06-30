# MMT 论文设计补充：SVD 与第二、三章边界

本补充文件经用户确认，覆盖主设计文档中第二章和第三章的对应描述。

## 第二章：Kinematic Modeling and Reconfiguration Analysis

小节结构：

1. Mechanism architecture and geometric parameters
2. Screw-system-based mobility analysis
3. Closed-loop kinematic formulation
4. SVD-based singularity analysis
5. Branch reconfiguration at singular configurations

SVD 小节必须给出：

\[
\mathbf J_p=\mathbf U\boldsymbol{\Sigma}\mathbf V^T,
\qquad
\boldsymbol{\Sigma}=\operatorname{diag}(\sigma_1,\ldots,\sigma_6).
\]

并解释：

- 被动雅可比矩阵的选取依据。
- 矩阵秩、最小奇异值与瞬时自由度之间的关系。
- \(\sigma_{\min}(\mathbf J_p)\rightarrow0\) 的奇异性判据。
- 真奇异位形与最小奇异值严格非零的病态或伪奇异位形之间的区别。
- 条件数对传动性能恶化的辅助解释。
- 奇异位形处瞬时自由度升高及驱动关节切换机制。

第二章中的图片用于解释方法和机理：

- 机构拓扑、初始位形、关节编号、坐标系和对称平面。
- SVD 奇异性识别流程。
- 正常位形与典型奇异位形对比。
- 奇异位形处两组正交关节轴。
- 空间机构退化为平面四杆机构的构型示意。

## 第三章：Numerical Validation of the Two Motion Branches

第三章完整保留，结构为：

1. Numerical continuation and singularity identification
2. Spatial symmetric branch
3. Degenerated planar branch and branch intersections

第三章集中呈现数值证据：

- BFGS 连续延拓方法和闭环误差阈值。
- 完整奇异值谱及最小奇异值随输入关节角变化的曲线。
- `0°/180°/360°` 附近的真实奇异位形。
- `90°/270°` 附近的病态或伪奇异状态。
- 空间分支各关节角曲线、对称约束图和 Fourier 拟合。
- 退化平面分支各关节角曲线、锁定关节及拟合。
- 两个运动分支的流形图和交点图。
- 关键奇异位形和误差数据表。

章节边界：

- 第二章回答“如何分析和判断”。
- 第三章回答“计算得到什么以及证据是否支持结论”。
- 第二章不提前堆放数值曲线，第三章不重复展开 SVD 基础理论。
