# MMT论文全局写作规划

## 1. 论文定位

目标期刊是 *Mechanism and Machine Theory*，因此本文应优先写成一篇机构学论文，而不是一篇完整的强化学习机器人控制论文。

建议主线：

> 提出一种 7R 平面对称空间闭链机构，证明其具有一个全局自由度、两个可通过奇异位形切换的单自由度运动分支，并将该可重构特性用于欠驱动移动机器人，实现多模态步态的仿真验证。

英文核心论点：

> This paper presents a plane-symmetric spatial 7R mechanism whose singular configurations act as deterministic gateways between two one-DOF motion branches, and demonstrates its use as an underactuated mobile robot with switchable locomotion modes.

## 2. 文章边界

本篇重点：

- 7R 平面对称空间机构的构型设计。
- 自由度、公共约束、奇异位形和分支切换分析。
- 两个单自由度运动分支的数值延拓、SVD 验证和运动关系拟合。
- 将机构实现为冗余驱动移动机器人。
- 用 PPO 神经网络控制验证冗余驱动可以激发理论预测的运动模式，并实现基本模式切换和基础移动能力。

本篇不建议展开：

- 完整转向控制。
- 复杂路径规划。
- 大规模地形自适应。
- 多策略状态机完整控制系统。
- 强化学习优于传统控制的泛化性大论断。

转向、路径跟踪、复杂地形适应可以作为下一篇论文的核心。

## 3. 推荐标题

最稳妥标题：

> Kinematic Branch Reconfiguration of a Plane-Symmetric Spatial 7R Mechanism for Underactuated Mobile Locomotion

偏机构学标题：

> Mobility, Singularity and Branch Reconfiguration of a Plane-Symmetric Spatial 7R Mechanism

偏机器人应用标题：

> A Reconfigurable Plane-Symmetric 7R Mechanism for Multi-Mode Underactuated Mobile Robots

## 4. 推荐贡献点

建议写成三条贡献：

1. A novel plane-symmetric spatial 7R mechanism is proposed, and its global mobility is shown to be one through screw-system and modified G-K analyses.

   中文解释：提出一种新的 7R 平面对称空间机构，并通过螺旋理论和改进 G-K 公式证明其全局自由度为 1。

2. The mechanism is shown to possess two deterministic one-DOF motion branches connected by singular configurations, which are identified by SVD analysis of the passive Jacobian.

   中文解释：证明该机构有两个确定的单自由度运动分支，并且它们通过奇异位形连接；奇异位形由被动雅可比矩阵的 SVD 分析识别。

3. A redundantly actuated robotic embodiment is developed, and PPO-based simulation verifies that the learned motions agree with the predicted kinematic branches and can support branch switching.

   中文解释：设计冗余驱动机器人样机，并用 PPO 仿真验证神经网络得到的运动模式与理论运动分支一致，可以支持模式切换。

## 5. 全文章节大纲（推荐 6 章结构）

推荐采用更紧凑的 6 章结构：

1. Introduction
2. Kinematic Modeling and Reconfiguration Analysis
3. Numerical Validation of the Two Motion Branches
4. Robotic Embodiment and PPO-Based Validation
5. Discussion
6. Conclusion

这样可以避免“机构构型单独成章内容过薄”的问题，也更符合 MMT 这类机构学论文常见的组织方式。

### 1. Introduction

写作目的：

- 引出空间闭链机构、对称机构、可重构机构和奇异位形分支切换。
- 说明已有 Bennett/Bricard/过约束机构研究的不足：许多工作关注运动学性质，但较少将奇异分支切换转化为可控的机器人运动模式。
- 引出本文提出的 7R 平面对称机构和欠驱动机器人验证。

建议结构：

1. 空间闭链机构和对称机构的重要性。
2. 可重构机构与奇异位形在多模式运动中的潜力。
3. 现有工作不足：机构分支分析和机器人控制应用之间仍有断层。
4. 本文贡献：机构设计、分支证明、机器人/PPO 验证。

### 2. Kinematic Modeling and Reconfiguration Analysis

这是全文核心章节。

机构构型不要单独成章，而是作为本章开头小节，自然引入后续自由度、奇异性和分支分析。

建议小节：

#### 2.1 Mechanism architecture and geometric parameters

写作目的：

- 清楚定义机构，不急着进入复杂推导。
- 给出关节编号、坐标系、D-H 参数、对称平面、初始位形。

建议包含：

- 一张综合主图：机构拓扑 + 关节编号 + 对称平面 + 坐标系 + 初始构型。
- 一张 D-H 参数表。
- 对称约束的文字定义。

注意：

- 不建议把“构型图”和“初始位形图”拆成独立章节。
- 图 1 尽量做成综合图，避免图过碎。
- 这一小节写完后立刻进入自由度与奇异性分析。

#### 2.2 Screw-system-based mobility analysis

内容：

- 由 D-H 参数得到关节螺旋。
- 构造机构雅可比矩阵。
- 求解 reciprocal screw system。
- 得到无公共约束，约束数为 0。
- 用改进 G-K 公式得到全局自由度为 1。

注意：

- 正文保留关键螺旋表达或结果。
- 过长的螺旋系矩阵可放附录。

#### 2.3 Closed-loop kinematic formulation

内容：

- 写闭环方程。
- 定义全局雅可比矩阵。
- 区分主动关节和被动关节。
- 给出被动雅可比矩阵。

核心公式：

```latex
T_1 T_2 T_3 T_4 T_5 T_6 T_7 = I
```

```latex
J \dot{q} = 0
```

```latex
J_p \dot{q}_p = -J_a \dot{q}_a
```

#### 2.4 SVD-based singularity identification

内容：

- 说明为什么用 SVD 判断奇异位形。
- 以关节 2 为驱动关节。
- 通过数值延拓追踪一个运动分支。
- 根据最小奇异值识别真实奇异点和伪奇异点。

重点：

- `90 deg` 和 `270 deg` 附近如果只是病态而不是实际奇异，要明确称为 pseudo-singular or ill-conditioned configurations。
- `0 deg` 和 `360 deg` 是同一物理构型，表述要严谨。

#### 2.5 Two one-DOF branches and transition conditions

内容：

- 空间对称分支：

```latex
q_7 = q_1,\quad q_6 = -q_2,\quad q_5 = -q_3
```

- 奇异位形瞬时自由度升高为 2。
- 机构轴线分成两组：`{2,4,6}` 与 `{1,3,5,7}`。
- 退化平面分支由 `{1,3,5,7}` 构成等效平面四连杆。
- 两个分支只在特定位形相交，因此分支切换是可预测的。

### 3. Numerical Validation of the Two Motion Branches

写作目的：

- 用图和数据证明第二章的理论结论。

建议图表：

- 最小奇异值随驱动角变化曲线。
- 空间分支中各关节角随 `q_2` 变化曲线。
- 平面分支中各关节角随 `q_1` 变化曲线。
- 两个运动分支的流形交点图。
- 关键奇异位形表格。

注意：

- 傅里叶拟合公式可保留一组核心表达，其余放附录。
- 章节重点不是“拟合公式很长”，而是“两个分支确实存在且只在特定位形连通”。

### 4. Robotic Embodiment and PPO-Based Validation

写作目的：

- 将抽象机构转成欠驱动移动机器人。
- 解释为什么需要冗余电机驱动。
- 用 PPO 仿真验证冗余驱动机器人能够激发理论预测的运动模式。

建议小节：

#### 4.1 Robotic embodiment and redundant actuation

写：

- 机器人 CAD 和样机结构。
- 四电机布置。
- 平面模式主动关节和空间模式主动关节。
- PLA-CF 和 3D 打印材料。
- 主要触地杆件。
- 冗余驱动的原因：单电机尺寸和扭矩不足，奇异位形附近传动性能恶化。

#### 4.2 Simulation and training setup

写：

- mjlab 仿真环境。
- PPO 算法。
- 网络结构。
- 环境数量。
- 最大迭代次数。
- 动作空间：四个电机力矩或目标角。
- 观测空间。
- 奖励函数。

#### 4.3 Redundant actuation groups and ablation design

写：

- 四个电机两两组合。
- 平面对称组合。
- 空间对称组合。
- 非对称组合。
- 同一随机种子和相同 PPO 参数用于对比。

#### 4.4 Emergence of kinematic branches

写：

- PPO 训练中出现空间运动模式和平面退化模式。
- 将仿真关节角关系与理论曲线对比。
- 计算相关系数或误差。
- 在奇异位形处代入雅可比，验证行列式或最小奇异值接近零。

#### 4.5 Branch switching behavior

写：

- 机器人可以通过改变驱动模式在奇异位形附近切换分支。
- 当前重点是前进、后退和基础移动，不展开完整转向。

建议避免：

- 不要说 PPO 完全解决了机器人控制。
- 不要说零样本泛化很强，除非有明确实验。
- 不要把转向作为必须完成的核心结果。

#### 4.6 Basic locomotion demonstration

写作目的：

- 展示机构分支在移动机器人上的直接运动意义。

建议保留：

- 空间整周回转爬行模式。
- 空间蠕动模式。
- 平面蠕动模式。
- 前进和后退能力。
- 基础速度、步长、效率指标。

可以选择性加入：

- 低矮空间通过。
- 简单越障。
- 受限环境下的被动降级现象。

建议暂不展开：

- 完整转向。
- 复杂路径跟踪。
- 地形自适应策略切换。

### 5. Discussion

建议讨论：

1. 机构意义：

   奇异位形不是单纯缺陷，而是可控的分支切换入口。

2. 控制意义：

   机构拓扑提前组织了运动空间，使 PPO 不需要从零学习任意高维运动，而是在机构允许的分支内寻找控制策略。

3. 应用意义：

   冗余驱动和分支切换为欠驱动移动机器人提供了多模态运动能力。

4. 局限：

   - 当前主要验证前进、后退和基础模式切换。
   - 转向控制和复杂地形适应尚未系统展开。
   - 接触、摩擦、冲击和载荷对实物性能的影响还需进一步研究。

可用英文句：

```text
Although the present study focuses on branch reconfiguration and straight-line locomotion, the same redundant actuation architecture can be extended to steering and terrain-adaptive gait transitions. These problems require additional contact-aware reward design and trajectory-level evaluation, and are therefore left for future work.
```

中文解释：

本文重点是分支可重构和直线移动；转向与地形适应需要额外的接触奖励设计和轨迹级评价，因此作为后续工作。

### 6. Conclusion

结论不要太长。

建议四句话：

1. 本文提出一种 7R 平面对称空间闭链机构。
2. 证明其全局自由度为 1，并存在两个由奇异位形连接的单自由度运动分支。
3. 数值延拓、SVD 和仿真验证了分支关系和切换条件。
4. 冗余驱动机器人和 PPO 控制验证了该机构在多模态欠驱动移动中的应用潜力。

## 6. 图表规划（按 6 章结构）

建议主文图：

1. 机构拓扑、关节编号、对称平面、坐标系、初始构型综合图。
2. D-H 参数表。
3. 最小奇异值随 `q_2` 变化曲线。
4. 空间主分支关节角关系曲线。
5. 主分支约束关系图：`q_7` vs `q_1`、`q_6` vs `-q_2`、`q_5` vs `-q_3`。
6. 退化平面分支关节角关系曲线。
7. 两个分支流形图与交点图。
8. 机器人 CAD 和实物图。
9. 电机布置和冗余驱动示意图。
10. PPO 奖励曲线和消融对比。
11. 理论关节曲线与 PPO 仿真关节数据对比。
12. 基础步态序列图与性能表。

建议附录内容：

- 完整 D-H 变换矩阵。
- 完整闭环方程 permutation。
- 过长螺旋系表达。
- 傅里叶拟合完整系数。
- PPO 详细参数表。
- 额外训练曲线和回放结果。

## 7. 当前主稿需要修改的问题

需要删除或替换：

- Elsevier 模板残留作者、邮箱、机构和 biography。
- 示例图片、测试表格、theorem 示例、旧关键词。
- 重复段落和没有真实数据支撑的占位内容。

需要统一：

- `q_i` 和 `theta_i` 的符号。
- `Joint 2`、`q_2`、`v_2` 等命名。
- 奇异位形表述：`0 deg` 和 `360 deg` 是同一物理构型。
- 空间分支和平面分支的约束关系。
- 主动关节和被动关节的定义。

需要补充：

- MMT 格式摘要，180 词以内。
- 3-5 条 Highlights，每条不超过 85 字符。
- 图表真实编号和 caption。
- 参考文献。
- PPO 参数表。
- 机器人性能指标表。

## 8. 为什么采用 6 章结构

采用 6 章结构的主要原因：

1. “机构构型”本身不足以单独撑起一整章，拆开会显得发散。
2. 对 MMT 而言，构型定义本来就是运动学分析的入口，合并更自然。
3. 机器人与 PPO 验证内容已经够多，适合保留为独立一章。
4. 这样全文会形成很清楚的主线：

`机构定义 -> 自由度与奇异性 -> 两分支验证 -> 机器人/PPO 验证 -> 讨论 -> 结论`

## 9. 下一篇论文方向

下一篇可以专门写：

> Reinforcement-learning-based steering and terrain-adaptive locomotion of a reconfigurable 7R mobile robot

核心内容：

- 转向控制。
- 路径跟踪。
- 多策略状态机。
- 复杂地形适应。
- 越障、低矮空间通过、狭窄通道。
- 接触感知奖励函数。
- 真实样机实验对比。

这样两篇论文边界清楚：

- 第一篇：机构学理论 + 基础机器人验证。
- 第二篇：强化学习控制 + 多地形运动能力。
