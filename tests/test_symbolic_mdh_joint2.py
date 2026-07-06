from __future__ import annotations

import importlib.util
import sys
from pathlib import Path


def load_module():
    """加载关节 2 的 MDH 推导脚本，便于直接调用其公开函数。"""
    module_path = Path(r"E:\hub\symbolic_mdh_joint2\derive_joint2_mdh.py")
    spec = importlib.util.spec_from_file_location("derive_joint2_mdh", module_path)
    module = importlib.util.module_from_spec(spec)
    assert spec is not None and spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_joint2_mdh_outputs_and_closure_error(tmp_path: Path):
    """验证脚本能输出完整文件，并且零位时闭环误差满足要求。"""
    module = load_module()
    output_dir = tmp_path / "symbolic_mdh_joint2"

    result = module.build_all_outputs(output_dir)

    expected_files = {
        "mdh_matrices.txt",
        "closure_matrix.txt",
        "block_equations.txt",
        "half_angle_equations.txt",
        "equation_summary.txt",
        "README.md",
    }
    actual_files = {path.name for path in output_dir.iterdir() if path.is_file()}

    assert expected_files.issubset(actual_files)
    assert len(result["A_matrices"]) == 7
    assert len(result["equations"]) == 16
    assert float(result["closure_error_fro"]) < 1e-12
