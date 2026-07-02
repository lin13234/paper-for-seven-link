# 七杆闭链空间运动分支显式解 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 基于给定改进型 DH 参数和三组对称约束，得到经过零位的一维七杆空间运动分支代数显式解、连续数值结果和完整闭环验证。

**Architecture:** 先建立与原脚本逐项一致的独立 DH 闭环模块，再用半角变量实现四次代数方程及分支连续选择。分析入口只调用前两个模块，负责跨半角坐标图延拓、误差审核、CSV/图片输出；Markdown 文档记录从矩阵闭环到代数参数化的完整推导。

**Tech Stack:** Python 3、NumPy、SciPy、Matplotlib、Pandas、Pytest；SymPy 只用于离线复核消元，不作为运行时求解依赖。

## Global Constraints

- 固定使用 \(l=1,\ a=2,\ b=5/2\) 及原脚本中的 \(\theta_3,\theta_6\) 的 \(\pi\) 偏置。
- 使用原脚本的改进型 DH 矩阵，闭环目标为 \(T_1\cdots T_7=I_4\)。
- 对称分支必须满足 \(q_5=-q_3,\ q_6=-q_2,\ q_7=q_1\)。
- 只选择经过 \(\mathbf q_0=\mathbf0\) 的连续实分支。
- 全矩阵误差 \(\|T-I_4\|_F<10^{-9}\) 才能标记为有效。
- Fourier 拟合不得作为精确显式解。
- 所有新增代码使用完整中文注释。
- 不修改已有 `planar_fourbar_dh/` 和相关测试。

---

### Task 1: 改进型 DH 闭环内核

**Files:**
- Create: `seven_bar_spatial_branch/__init__.py`
- Create: `seven_bar_spatial_branch/kinematics.py`
- Create: `tests/test_seven_bar_spatial_branch.py`

**Interfaces:**
- Produces: `modified_dh_transform(alpha, a, d, theta) -> np.ndarray`
- Produces: `symmetric_joint_vector(q1, q2, q3, q4) -> np.ndarray`
- Produces: `closure_matrix(q) -> np.ndarray`
- Produces: `closure_errors(q) -> dict[str, float]`

- [ ] **Step 1: 写零位与对称映射失败测试**

```python
import numpy as np

from seven_bar_spatial_branch.kinematics import (
    closure_errors,
    closure_matrix,
    symmetric_joint_vector,
)


def test_zero_configuration_is_closed():
    q = np.zeros(7)
    assert np.allclose(closure_matrix(q), np.eye(4), atol=1e-12)
    assert closure_errors(q)["matrix"] < 1e-12


def test_symmetric_joint_vector_uses_confirmed_relations():
    q = symmetric_joint_vector(0.1, 0.2, -0.3, 0.4)
    expected = np.array([0.1, 0.2, -0.3, 0.4, 0.3, -0.2, 0.1])
    assert np.allclose(q, expected, atol=0.0)
```

- [ ] **Step 2: 运行测试并确认因模块不存在而失败**

Run: `python -m pytest tests/test_seven_bar_spatial_branch.py -v`

Expected: FAIL，错误包含 `ModuleNotFoundError: No module named 'seven_bar_spatial_branch'`。

- [ ] **Step 3: 实现最小 DH 内核**

`seven_bar_spatial_branch/kinematics.py` 必须实现：

```python
import numpy as np


DH_PARAMETERS = (
    (0.0, 1.0, 0.0, 0.0),
    (np.pi / 2, 2.0, 2.5, 0.0),
    (np.pi / 2, 0.0, 0.0, np.pi),
    (np.pi / 2, 2.5, 0.0, 0.0),
    (np.pi / 2, 2.5, 0.0, 0.0),
    (np.pi / 2, 0.0, 2.5, np.pi),
    (np.pi / 2, 2.0, 0.0, 0.0),
)


def modified_dh_transform(alpha, a, d, theta):
    """返回与原始脚本完全一致的改进型 DH 齐次变换矩阵。"""
    ca, sa = np.cos(alpha), np.sin(alpha)
    ct, st = np.cos(theta), np.sin(theta)
    return np.array([
        [ct, -st, 0.0, a],
        [st * ca, ct * ca, -sa, -d * sa],
        [st * sa, ct * sa, ca, d * ca],
        [0.0, 0.0, 0.0, 1.0],
    ])


def symmetric_joint_vector(q1, q2, q3, q4):
    """由四个降维变量构造满足已确认对称关系的七关节向量。"""
    return np.array([q1, q2, q3, q4, -q3, -q2, q1], dtype=float)


def closure_matrix(q):
    """依次右乘七个 DH 矩阵，返回闭环末端齐次变换。"""
    q = np.asarray(q, dtype=float)
    if q.shape != (7,):
        raise ValueError("q 必须是长度为 7 的一维数组")
    transform = np.eye(4)
    for qi, (alpha, a, d, offset) in zip(q, DH_PARAMETERS):
        transform = transform @ modified_dh_transform(alpha, a, d, qi + offset)
    return transform


def closure_errors(q):
    """返回位置、姿态和完整矩阵三种闭环误差。"""
    transform = closure_matrix(q)
    rotation = transform[:3, :3]
    cosine = np.clip((np.trace(rotation) - 1.0) / 2.0, -1.0, 1.0)
    return {
        "position": float(np.linalg.norm(transform[:3, 3])),
        "orientation": float(np.arccos(cosine)),
        "matrix": float(np.linalg.norm(transform - np.eye(4), ord="fro")),
    }
```

- [ ] **Step 4: 运行 Task 1 测试**

Run: `python -m pytest tests/test_seven_bar_spatial_branch.py -v`

Expected: 2 passed。

- [ ] **Step 5: 提交 Task 1**

```bash
git add seven_bar_spatial_branch/__init__.py seven_bar_spatial_branch/kinematics.py tests/test_seven_bar_spatial_branch.py
git commit -m "feat: add seven-bar DH closure kernel"
```

---

### Task 2: 半角代数方程和单点显式解

**Files:**
- Create: `seven_bar_spatial_branch/explicit_solution.py`
- Modify: `tests/test_seven_bar_spatial_branch.py`

**Interfaces:**
- Consumes: `symmetric_joint_vector`, `closure_errors`
- Produces: `quartic_coefficients(t) -> np.ndarray`
- Produces: `candidate_y_roots(t, imag_tol=1e-9) -> np.ndarray`
- Produces: `solve_explicit_at_t(t, previous=None) -> SpatialSolution`
- Produces: `SpatialSolution(t, x, y, z, q, algebraic_residual, closure_error)`

- [ ] **Step 1: 写四次方程与典型构型失败测试**

```python
from seven_bar_spatial_branch.explicit_solution import (
    candidate_y_roots,
    quartic_polynomial,
    solve_explicit_at_t,
)


def test_zero_half_angle_returns_zero_configuration():
    solution = solve_explicit_at_t(0.0)
    assert np.allclose(solution.q, np.zeros(7), atol=1e-12)
    assert solution.closure_error < 1e-12


def test_quartic_root_and_full_closure_at_q2_pi_over_four():
    t = np.tan(np.pi / 8)
    solution = solve_explicit_at_t(t)
    assert abs(quartic_polynomial(t, solution.y)) < 1e-10
    assert solution.closure_error < 1e-10
    assert np.isclose(solution.q[1], np.pi / 4, atol=1e-12)


def test_candidate_roots_are_real_and_satisfy_polynomial():
    t = 0.5
    roots = candidate_y_roots(t)
    assert len(roots) >= 1
    assert all(abs(quartic_polynomial(t, root)) < 1e-9 for root in roots)
```

- [ ] **Step 2: 运行新增测试并确认因接口不存在而失败**

Run: `python -m pytest tests/test_seven_bar_spatial_branch.py -v`

Expected: FAIL，错误包含 `No module named 'seven_bar_spatial_branch.explicit_solution'`。

- [ ] **Step 3: 实现四次多项式和候选根**

四次式按 \(y^4\) 到常数项排列：

```python
def quartic_coefficients(t):
    """返回 F(t,y) 关于 y 的降幂系数。"""
    t2 = t * t
    t4 = t2 * t2
    return np.array([
        5.0 * t2 - 20.0,
        5.0 * t4 - 50.0 * t2 + 45.0,
        -29.0 * t4 + 88.0 * t2 - 29.0,
        45.0 * t4 - 50.0 * t2 + 5.0,
        -20.0 * t4 + 5.0 * t2,
    ])


def quartic_polynomial(t, y):
    """计算消元四次方程 F(t,y)。"""
    return float(np.polyval(quartic_coefficients(t), y))


def candidate_y_roots(t, imag_tol=1e-9):
    """只保留虚部处于舍入误差范围内的实代数根。"""
    roots = np.roots(quartic_coefficients(t))
    real_roots = [root.real for root in roots if abs(root.imag) <= imag_tol]
    return np.array(sorted(real_roots), dtype=float)
```

- [ ] **Step 4: 实现 \(x,z\) 恢复和单点分支选择**

实现 `SpatialSolution` 数据类。`solve_explicit_at_t` 在 `t=0` 返回零位；普通点对每个 \(y\) 候选根计算：

\[
z=\frac{2t(1-y^2)}{(t^2-1)(1+y^2)}
\]

和 \(x\) 二次方程的两个实根，将所有候选七关节向量代入完整闭环。无 `previous` 时，以

\[
|y+t^2|+|x+t^2|+\|T-I_4\|_F
\]

最小者作为经过零位的局部分支；有 `previous` 时，以包角后的关节距离和闭环误差共同排序。若最优候选闭环误差大于 \(10^{-9}\)，抛出 `RuntimeError`。

- [ ] **Step 5: 运行 Task 2 测试**

Run: `python -m pytest tests/test_seven_bar_spatial_branch.py -v`

Expected: 5 passed。

- [ ] **Step 6: 提交 Task 2**

```bash
git add seven_bar_spatial_branch/explicit_solution.py tests/test_seven_bar_spatial_branch.py
git commit -m "feat: add algebraic spatial branch solution"
```

---

### Task 3: 连续延拓、坐标奇点和一维性验证

**Files:**
- Modify: `seven_bar_spatial_branch/explicit_solution.py`
- Modify: `seven_bar_spatial_branch/kinematics.py`
- Modify: `tests/test_seven_bar_spatial_branch.py`

**Interfaces:**
- Produces: `trace_branch(q2_values) -> list[SpatialSolution]`
- Produces: `reduced_closure_jacobian(q_reduced, step=1e-7) -> np.ndarray`
- Produces: `reduced_jacobian_rank(q_reduced, tolerance=1e-7) -> int`

- [ ] **Step 1: 写跨越 \(q_2=\pi/2\) 和局部秩失败测试**

```python
from seven_bar_spatial_branch.explicit_solution import trace_branch
from seven_bar_spatial_branch.kinematics import reduced_jacobian_rank


def test_trace_crosses_t_equal_one_without_branch_jump():
    q2_values = np.linspace(1.45, 1.70, 31)
    solutions = trace_branch(q2_values)
    q_values = np.array([item.q for item in solutions])
    assert len(solutions) == len(q2_values)
    assert max(item.closure_error for item in solutions) < 1e-9
    assert np.max(np.abs(np.diff(q_values, axis=0))) < 0.2


def test_symmetric_reduced_closure_has_one_dimensional_nullspace():
    solution = trace_branch(np.array([0.6, 0.8]))[-1]
    reduced = solution.q[[0, 1, 2, 3]]
    assert reduced_jacobian_rank(reduced) == 3
```

- [ ] **Step 2: 运行新增测试并确认失败**

Run: `python -m pytest tests/test_seven_bar_spatial_branch.py -v`

Expected: FAIL，错误指出 `trace_branch` 或 `reduced_jacobian_rank` 未定义。

- [ ] **Step 3: 实现完整矩阵残差与数值雅可比**

`kinematics.py` 新增：

```python
def closure_residual_12(q):
    """展平旋转矩阵误差和位置误差，避免 π 转角漏检。"""
    transform = closure_matrix(q)
    return (transform[:3, :] - np.eye(4)[:3, :]).reshape(-1)


def reduced_closure_jacobian(q_reduced, step=1e-7):
    """对四个对称降维变量计算中心差分雅可比。"""
    q_reduced = np.asarray(q_reduced, dtype=float)
    jacobian = np.empty((12, 4))
    for column in range(4):
        delta = np.zeros(4)
        delta[column] = step
        plus = closure_residual_12(symmetric_joint_vector(*(q_reduced + delta)))
        minus = closure_residual_12(symmetric_joint_vector(*(q_reduced - delta)))
        jacobian[:, column] = (plus - minus) / (2.0 * step)
    return jacobian


def reduced_jacobian_rank(q_reduced, tolerance=1e-7):
    """返回对称降维闭环雅可比的数值秩。"""
    singular_values = np.linalg.svd(
        reduced_closure_jacobian(q_reduced), compute_uv=False
    )
    return int(np.count_nonzero(singular_values > tolerance))
```

- [ ] **Step 4: 实现连续延拓和 \(t^2=1\) 处理**

`trace_branch` 逐点处理输入角。普通点调用 `solve_explicit_at_t`；当

\[
|t^2-1|<10^{-6}
\]

时，用前一点作为初值，通过 `scipy.optimize.least_squares` 求解 \((q_1,q_3,q_4)\)，残差使用完整 12 分量闭环误差。每点必须验证对称关系和矩阵误差。角度输出通过相邻点加减 \(2\pi\) 保持连续。

- [ ] **Step 5: 运行 Task 3 测试**

Run: `python -m pytest tests/test_seven_bar_spatial_branch.py -v`

Expected: 7 passed。

- [ ] **Step 6: 提交 Task 3**

```bash
git add seven_bar_spatial_branch/explicit_solution.py seven_bar_spatial_branch/kinematics.py tests/test_seven_bar_spatial_branch.py
git commit -m "feat: trace spatial branch through chart singularities"
```

---

### Task 4: 分析入口和结果文件

**Files:**
- Create: `seven_bar_spatial_branch/run_analysis.py`
- Create: `seven_bar_spatial_branch/README.md`
- Create: `tests/test_seven_bar_analysis.py`

**Interfaces:**
- Consumes: `trace_branch`, `closure_errors`, `reduced_jacobian_rank`
- Produces: `solution_dataframe(solutions) -> pandas.DataFrame`
- Produces: CLI `python -m seven_bar_spatial_branch.run_analysis --output <dir>`

- [ ] **Step 1: 写数据表结构失败测试**

```python
import numpy as np

from seven_bar_spatial_branch.explicit_solution import trace_branch
from seven_bar_spatial_branch.run_analysis import solution_dataframe


def test_solution_dataframe_contains_angles_and_audit_errors():
    frame = solution_dataframe(trace_branch(np.linspace(0.0, 0.8, 5)))
    required = {
        "q2_input_rad", "q1_rad", "q2_rad", "q3_rad", "q4_rad",
        "q5_rad", "q6_rad", "q7_rad", "algebraic_residual",
        "position_error", "orientation_error", "matrix_error",
    }
    assert required.issubset(frame.columns)
    assert frame["matrix_error"].max() < 1e-9
```

- [ ] **Step 2: 运行测试并确认因入口不存在而失败**

Run: `python -m pytest tests/test_seven_bar_analysis.py -v`

Expected: FAIL，错误包含 `No module named 'seven_bar_spatial_branch.run_analysis'`。

- [ ] **Step 3: 实现 DataFrame 与命令行入口**

`solution_dataframe` 将每个 `SpatialSolution` 展开为一行，同时重新调用 `closure_errors`，不能复用未经核查的缓存误差。CLI 默认生成 721 个 \(q_2\in[0,2\pi]\) 采样点，并写出：

- `spatial_branch_solutions.csv`
- `spatial_branch_joint_angles.png`
- `spatial_branch_closure_errors.png`
- `typical_solutions.md`

若任一点矩阵误差超过 \(10^{-9}\)，程序以非零状态退出，不写“成功”摘要。

- [ ] **Step 4: 编写中文 README**

README 必须包含运行命令、DH 约定、对称关系、半角变量定义、四次式、分支选择规则、坐标奇点说明和各输出文件用途。

- [ ] **Step 5: 运行分析测试与小规模 CLI**

Run: `python -m pytest tests/test_seven_bar_analysis.py -v`

Expected: 1 passed。

Run: `python -m seven_bar_spatial_branch.run_analysis --steps 41 --q2-end 1.4 --output tmp/seven_bar_smoke`

Expected: exit 0，四个输出文件存在，最大矩阵误差小于 \(10^{-9}\)。

- [ ] **Step 6: 提交 Task 4**

```bash
git add seven_bar_spatial_branch/run_analysis.py seven_bar_spatial_branch/README.md tests/test_seven_bar_analysis.py
git commit -m "feat: export seven-bar spatial branch analysis"
```

---

### Task 5: 严格推导文档与全量验证

**Files:**
- Create: `03推导分析/七杆空间运动分支/七杆空间运动分支显式解推导.md`
- Generate: `03推导分析/七杆空间运动分支/数值结果/`
- Modify: `tests/test_seven_bar_spatial_branch.py`

**Interfaces:**
- Consumes: 所有前述公开接口和分析输出
- Produces: 可复算中文推导、典型数值解表、全周期 CSV 与图片

- [ ] **Step 1: 增加代表构型参数化测试**

```python
import pytest


@pytest.mark.parametrize(
    "q2",
    [0.0, np.pi / 6, np.pi / 4, np.pi / 2, 3 * np.pi / 4],
)
def test_representative_configurations_close(q2):
    solutions = trace_branch(np.linspace(0.0, q2, max(2, int(q2 * 40) + 2)))
    assert solutions[-1].closure_error < 1e-9
```

- [ ] **Step 2: 运行代表构型测试**

Run: `python -m pytest tests/test_seven_bar_spatial_branch.py -v`

Expected: 全部通过；若 \(q_2=\pi/2\) 失败，必须修正坐标奇点处理，禁止放宽闭环阈值。

- [ ] **Step 3: 写严格推导文档**

文档按以下固定顺序展开：

1. 原始改进型 DH 矩阵与七组参数；
2. 零位闭环；
3. 三组对称关系及四变量降维；
4. 半角代换；
5. 姿态闭环的两个独立因子；
6. 由其中一式解出 \(z\)；
7. 位置闭环与结果式消元；
8. 四次方程 \(F(t,y)=0\)；
9. \(x\) 的二次方程和符号选择；
10. 七关节角恢复；
11. \(t^2=1\) 与 \(t=\infty\) 的坐标图问题；
12. 一维性、闭环误差和数值交叉验证；
13. 代表构型数值表；
14. Fourier 拟合与精确显式解的边界。

每个关键等式都注明来源是“姿态闭环”“位置闭环”还是“结果式消元”，避免直接罗列最终公式。

- [ ] **Step 4: 运行全量分析**

Run:

```bash
python -m seven_bar_spatial_branch.run_analysis --steps 721 --q2-end 6.283185307179586 --output "03推导分析/七杆空间运动分支/数值结果"
```

Expected: exit 0；CSV 共 721 行；所有矩阵误差小于 \(10^{-9}\)；不存在未说明的分支跳变。

- [ ] **Step 5: 运行完整测试**

Run: `python -m pytest tests/test_seven_bar_spatial_branch.py tests/test_seven_bar_analysis.py -v`

Expected: 0 failed。

- [ ] **Step 6: 检查工作区差异**

Run: `git diff --check`

Expected: 无输出，exit 0。

- [ ] **Step 7: 提交推导和最终结果**

```bash
git add seven_bar_spatial_branch tests/test_seven_bar_spatial_branch.py tests/test_seven_bar_analysis.py "03推导分析/七杆空间运动分支"
git commit -m "docs: derive and verify seven-bar spatial branch"
```

## Self-review

- 设计说明第 1—9 节均映射到 Task 1—5。
- 所有生产函数在对应任务中先有失败测试。
- 公开接口名称在各任务之间保持一致。
- 完整闭环使用 12 分量矩阵残差，不使用可能漏检 \(\pi\) 转角的反对称向量作为唯一判据。
- \(q_2=\pi/2\) 的 \(t^2=1\) 和 \(q_2=\pi\) 的半角无穷均有明确处理路径。
- 没有使用占位实现或把 Fourier 拟合列为显式解。
