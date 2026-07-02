# Planar Four-Bar DH Explicit Solution Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 生成一个可直接运行的 Python 工程，用解析闭环求出 \(q_4(q_2),q_6(q_2),q_1(q_2)\)，并用完整七杆 DH 矩阵验证和绘图。

**Architecture:** 将纯运动学计算、命令行绘图入口和测试分开。核心模块先从 DH 参数提取四杆轴心与杆长，再用两圆交点解析求装配支路，最后通过完整 DH 闭环残差验证角度映射。

**Tech Stack:** Python 3、NumPy、Matplotlib、pytest；不依赖 SciPy 优化器。

## Global Constraints

- 初始关节变量固定为 `q0 = np.zeros(7)`。
- 锁死 `q3 = q5 = q7 = 0`，以 `q2` 为唯一输入。
- 保持经过初始构型的装配支路，不用傅里叶或多项式拟合代替显式解。
- 所有角度计算以弧度进行，对外表格和主图使用度。
- 图像仅用 Python/Matplotlib 生成，输出 PNG、SVG、PDF。
- 代码包含中文注释，明确公式来源、变量含义和支路处理。

---

### Task 1: DH 闭环与四杆几何提取

**Files:**
- Create: `planar_fourbar_dh/fourbar_dh.py`
- Create: `tests/test_fourbar_dh.py`

**Interfaces:**
- Produces: `dh_transform(alpha, a, d, theta) -> ndarray`
- Produces: `forward_kinematics(q, l=1.0) -> list[ndarray]`
- Produces: `joint_axis_geometry(q, l=1.0) -> tuple[ndarray, ndarray]`
- Produces: `extract_fourbar_geometry(l=1.0) -> FourBarGeometry`

- [ ] **Step 1: 写 DH 初始闭环失败测试**

```python
def test_zero_configuration_is_closed():
    transforms = forward_kinematics(np.zeros(7))
    np.testing.assert_allclose(transforms[-1], np.eye(4), atol=1e-12)
```

- [ ] **Step 2: 运行测试并确认因模块不存在而失败**

Run: `python -m pytest tests/test_fourbar_dh.py::test_zero_configuration_is_closed -v`

Expected: `ModuleNotFoundError: No module named 'planar_fourbar_dh'`

- [ ] **Step 3: 实现 DH 变换和正运动学**

实现原脚本给定的七组参数，并返回从单位阵到末端的全部累计变换。

- [ ] **Step 4: 写四根轴平行及杆长失败测试**

```python
def test_active_axes_and_link_lengths_at_zero():
    geometry = extract_fourbar_geometry()
    expected = np.array([np.sqrt(41) / 2, 5.0, np.sqrt(41) / 2, 1.0])
    np.testing.assert_allclose(geometry.lengths, expected, atol=1e-12)
    assert geometry.axes_are_parallel
```

- [ ] **Step 5: 实现改进 DH 关节轴计算和几何提取**

实际关节轴方向按

```python
axis = R_previous @ np.array([0.0, -np.sin(alpha), np.cos(alpha)])
point = (T_previous @ np.array([a, 0.0, 0.0, 1.0]))[:3]
```

计算，提取关节 `1, 3, 5, 7` 的轴心和相邻距离。

- [ ] **Step 6: 运行 Task 1 测试**

Run: `python -m pytest tests/test_fourbar_dh.py -v`

Expected: 所有现有测试通过。

---

### Task 2: 同一装配支路的显式解与 DH 角度回映射

**Files:**
- Modify: `planar_fourbar_dh/fourbar_dh.py`
- Modify: `tests/test_fourbar_dh.py`

**Interfaces:**
- Produces: `circle_intersections(center_a, radius_a, center_b, radius_b) -> tuple[ndarray, ndarray]`
- Produces: `solve_explicit(q2_values, l=1.0, branch=None) -> FourBarSolution`
- `FourBarSolution` 包含 `q`, `points`, `reachable`, `position_error`, `rotation_error`

- [ ] **Step 1: 写初始支路恢复失败测试**

```python
def test_explicit_solution_recovers_zero_configuration():
    solution = solve_explicit(np.array([0.0]))
    np.testing.assert_allclose(solution.q[0], np.zeros(7), atol=1e-11)
    np.testing.assert_allclose(solution.points[0, 2], [-2.0, -2.5], atol=1e-11)
```

- [ ] **Step 2: 运行并确认因显式求解器缺失而失败**

Run: `python -m pytest tests/test_fourbar_dh.py::test_explicit_solution_recovers_zero_configuration -v`

Expected: `ImportError` 或 `AttributeError`，指出 `solve_explicit` 尚不存在。

- [ ] **Step 3: 实现两圆交点和初始支路选择**

使用设计文档中的

```python
x = (r_a**2 - r_b**2 + distance**2) / (2.0 * distance)
h2 = r_a**2 - x**2
intersection = center_a + x * e + sign * np.sqrt(h2) * e_perp
```

并通过与初始 \(P_6\) 的距离选择 `sign`。

- [ ] **Step 4: 写完整转动周期闭环失败测试**

```python
def test_full_revolution_satisfies_seven_joint_dh_closure():
    q2 = np.linspace(0.0, 2.0 * np.pi, 361)
    solution = solve_explicit(q2)
    assert np.all(solution.reachable)
    assert np.max(solution.position_error) < 1e-10
    assert np.max(solution.rotation_error) < 1e-10
```

- [ ] **Step 5: 从杆件绝对方向映射 DH 关节变量**

先按初始方向构造相对杆角候选，再根据关节轴方向符号得到 \(q_4,q_6,q_1\)。将候选代回完整 DH 矩阵，以闭环残差最小且角度连续者作为正确映射。

- [ ] **Step 6: 写锁死关节与连续性测试**

```python
def test_locked_joints_stay_zero_and_angles_are_continuous():
    solution = solve_explicit(np.linspace(0.0, 2.0 * np.pi, 721))
    np.testing.assert_allclose(solution.q[:, [1, 3, 5]], 0.0, atol=0.0)
    assert np.max(np.abs(np.diff(solution.q[:, [2, 4, 6]], axis=0))) < 0.2
```

- [ ] **Step 7: 运行全部核心测试**

Run: `python -m pytest tests/test_fourbar_dh.py -v`

Expected: 全部通过。

---

### Task 3: CSV、图像与命令行入口

**Files:**
- Create: `planar_fourbar_dh/run_analysis.py`
- Create: `tests/test_run_analysis.py`

**Interfaces:**
- Consumes: `solve_explicit`
- Produces: `write_csv(solution, path) -> None`
- Produces: `plot_results(solution, output_dir) -> list[Path]`
- Produces: 命令 `python -m planar_fourbar_dh.run_analysis`

- [ ] **Step 1: 写输出文件失败测试**

```python
def test_analysis_writes_expected_outputs(tmp_path):
    outputs = run_analysis(output_dir=tmp_path, steps=181)
    expected = {
        "joint_angles.csv",
        "joint_angle_mapping.png",
        "joint_angle_mapping.svg",
        "joint_angle_mapping.pdf",
        "closure_error.png",
        "fourbar_configurations.png",
    }
    assert expected.issubset({path.name for path in outputs})
    assert all(path.stat().st_size > 0 for path in outputs)
```

- [ ] **Step 2: 运行并确认入口模块不存在**

Run: `python -m pytest tests/test_run_analysis.py::test_analysis_writes_expected_outputs -v`

Expected: `ModuleNotFoundError` 或缺少 `run_analysis`。

- [ ] **Step 3: 实现 CSV 和三类图**

主图绘制 \(q_4,q_6,q_1\) 随 \(q_2\) 的连续变化；误差图使用对数纵轴；构型图选择 \(0^\circ,90^\circ,180^\circ,270^\circ\) 四个输入位置。

- [ ] **Step 4: 实现命令行参数**

支持：

```text
--output-dir
--steps
--q2-start-deg
--q2-end-deg
--length-scale
```

默认输出到 `planar_fourbar_output`，默认输入范围 `0°–360°`。

- [ ] **Step 5: 运行绘图测试**

Run: `python -m pytest tests/test_run_analysis.py -v`

Expected: 全部通过，且 Matplotlib 使用非交互后端。

---

### Task 4: 完整验证和使用说明

**Files:**
- Create: `planar_fourbar_dh/README.md`
- Modify: `tests/test_fourbar_dh.py`

**Interfaces:**
- Consumes: 全部公开函数和命令行入口
- Produces: 可复现实验命令、公式说明和输出文件解释

- [ ] **Step 1: 添加显式公式数值一致性测试**

在多个输入角下独立检查四条杆长约束，并断言误差小于 `1e-11`。

- [ ] **Step 2: 运行完整测试**

Run: `python -m pytest tests -v`

Expected: 全部测试通过，无失败和警告。

- [ ] **Step 3: 执行实际分析**

Run: `python -m planar_fourbar_dh.run_analysis --output-dir planar_fourbar_output --steps 721`

Expected: 输出 CSV、三格式主图、闭环误差图和构型图。

- [ ] **Step 4: 检查输出数值**

确认终端打印：

```text
最大位置闭环误差
最大姿态闭环误差
有效输入角样本数
装配支路符号
```

且两类闭环误差均低于 `1e-10`。

- [ ] **Step 5: 视觉检查**

检查 PNG：中文或英文标签无乱码、曲线无非物理跳变、图例不遮挡数据、误差图没有非法对数值、四个构型比例一致。

- [ ] **Step 6: 编写 README**

说明安装依赖、运行命令、显式公式、DH 角与平面杆角的区别、输出文件和支路定义。

- [ ] **Step 7: 最终回归**

Run: `python -m pytest tests -v`

Expected: 全部通过。
