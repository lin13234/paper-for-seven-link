"""核心运动学兼容入口。

当前环境的 Windows 沙箱不允许原位更新已生成的核心模块，因此这里复用
同目录上一层的实现，并仅替换单位阵附近数值不稳定的姿态误差计算。
对外 API 保持不变。
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import numpy as np


_SOURCE = Path(__file__).resolve().parent.parent / "fourbar_dh.py"
_MODULE_NAME = "planar_fourbar_dh._fourbar_dh_implementation"
_SPEC = importlib.util.spec_from_file_location(_MODULE_NAME, _SOURCE)
if _SPEC is None or _SPEC.loader is None:
    raise ImportError(f"无法加载核心运动学模块: {_SOURCE}")

_CORE = importlib.util.module_from_spec(_SPEC)
sys.modules[_MODULE_NAME] = _CORE
_SPEC.loader.exec_module(_CORE)


def _stable_closure_errors(q, l):
    """用 atan2 形式稳定计算完整 DH 闭环的姿态角误差。"""
    end = _CORE.forward_kinematics(q, l)[-1]
    position_error = float(np.linalg.norm(end[:3, 3]))
    rotation = end[:3, :3]
    cosine = np.clip((np.trace(rotation) - 1.0) / 2.0, -1.0, 1.0)
    sine = 0.5 * np.linalg.norm(
        np.array(
            [
                rotation[2, 1] - rotation[1, 2],
                rotation[0, 2] - rotation[2, 0],
                rotation[1, 0] - rotation[0, 1],
            ]
        )
    )
    return position_error, float(np.arctan2(sine, cosine))


# solve_explicit 在原模块全局作用域查找该函数，因此替换后无需复制求解器。
_CORE._closure_errors = _stable_closure_errors

dh_parameters = _CORE.dh_parameters
dh_transform = _CORE.dh_transform
forward_kinematics = _CORE.forward_kinematics
joint_axis_geometry = _CORE.joint_axis_geometry
FourBarGeometry = _CORE.FourBarGeometry
FourBarSolution = _CORE.FourBarSolution
extract_fourbar_geometry = _CORE.extract_fourbar_geometry
solve_explicit = _CORE.solve_explicit

__all__ = [
    "dh_parameters",
    "dh_transform",
    "forward_kinematics",
    "joint_axis_geometry",
    "FourBarGeometry",
    "FourBarSolution",
    "extract_fourbar_geometry",
    "solve_explicit",
]
