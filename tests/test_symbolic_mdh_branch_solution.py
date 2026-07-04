from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

import sympy as sp


MODULE_PATH = Path(__file__).resolve().parents[1] / "symbolic_mdh_branch_solution" / "derive_mdh_symbolic.py"


def load_module():
    spec = spec_from_file_location("derive_mdh_symbolic", MODULE_PATH)
    module = module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_mdh_matrix_matches_expected_structure():
    """MDH 单节变换必须严格符合 Rx*Tx*Rz*Tz 定义。"""
    module = load_module()
    alpha, a, d, theta = sp.symbols("alpha a d theta")
    actual = sp.simplify(module.mdh_matrix(alpha, a, d, theta))
    expected = sp.Matrix(
        [
            [sp.cos(theta), -sp.sin(theta), 0, a],
            [sp.cos(alpha) * sp.sin(theta), sp.cos(alpha) * sp.cos(theta), -sp.sin(alpha), -d * sp.sin(alpha)],
            [sp.sin(alpha) * sp.sin(theta), sp.sin(alpha) * sp.cos(theta), sp.cos(alpha), d * sp.cos(alpha)],
            [0, 0, 0, 1],
        ]
    )
    assert sp.simplify(actual - expected) == sp.zeros(4)


def test_zero_configuration_is_closed():
    """给定的对称初值必须满足完整 7R 闭环。"""
    module = load_module()
    transforms, total = module.build_symbolic_chain()
    assert len(transforms) == 7
    closed = sp.simplify(total.subs({module.x: 0, module.y: 0, module.z: 0, module.s: 0, module.l: 1}) - sp.eye(4))
    assert closed == sp.zeros(4)


def test_half_angle_substitution_produces_nonzero_polynomials():
    """闭环矩阵经半角代换后应能提取出非零分子多项式。"""
    module = load_module()
    _, total = module.build_symbolic_chain()
    closure = sp.simplify(total - sp.eye(4))
    polynomials = module.extract_unique_closure_polynomials(closure)
    assert polynomials
    assert all(poly.free_symbols <= {module.X, module.Y, module.Z, module.q, module.l} for poly in polynomials)
