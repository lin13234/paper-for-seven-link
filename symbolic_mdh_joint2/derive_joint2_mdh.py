from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Tuple

import sympy as sp


# 统一定义本项目中会反复使用的符号变量。
l = sp.symbols("l", real=True, nonzero=True)
x, y, z, s = sp.symbols("x y z s", real=True)
X, Y, Z, q = sp.symbols("X Y Z q", real=True)


@dataclass(frozen=True)
class EquationReport:
    """保存单个 block 方程在半角代换后的完整信息。"""

    name: str
    trig_expression: sp.Expr
    half_angle_expression: sp.Expr
    numerator: sp.Expr
    denominator: sp.Expr
    factored_numerator: sp.Expr
    total_degree: int
    variable_presence: Dict[str, bool]


def mdh_matrix(alpha_prev: sp.Expr, a_prev: sp.Expr, theta_i: sp.Expr, d_i: sp.Expr) -> sp.Matrix:
    """
    按照 Modified DH 定义构造单节齐次变换矩阵：
    A_i = Rx(alpha_{i-1}) * Tx(a_{i-1}) * Rz(theta_i) * Tz(d_i)
    """
    cos_alpha = sp.cos(alpha_prev)
    sin_alpha = sp.sin(alpha_prev)
    cos_theta = sp.cos(theta_i)
    sin_theta = sp.sin(theta_i)

    return sp.Matrix(
        [
            [cos_theta, -sin_theta, 0, a_prev],
            [sin_theta * cos_alpha, cos_theta * cos_alpha, -sin_alpha, -d_i * sin_alpha],
            [sin_theta * sin_alpha, cos_theta * sin_alpha, cos_alpha, d_i * cos_alpha],
            [0, 0, 0, 1],
        ]
    )


def build_joint_matrices() -> List[sp.Matrix]:
    """按 Joint1~Joint7 的原始编号构造 7 个 MDH 矩阵。"""
    half_pi = sp.pi / 2
    five_l_over_two = sp.Rational(5, 2) * l

    joint_parameters = [
        (0, l, x, 0),
        (half_pi, 2 * l, s, five_l_over_two),
        (half_pi, 0, sp.pi + y, 0),
        (half_pi, five_l_over_two, z, 0),
        (half_pi, five_l_over_two, -y, 0),
        (half_pi, 0, sp.pi - s, five_l_over_two),
        (half_pi, 2 * l, x, 0),
    ]

    return [mdh_matrix(*params) for params in joint_parameters]


def matrix_to_code(name: str, matrix: sp.Matrix) -> str:
    """把矩阵转成可直接复制的 SymPy 代码字符串。"""
    return f"{name} = sp.Matrix({sp.srepr(matrix.tolist())})"


def matrix_block_text(name: str, matrix: sp.Matrix) -> str:
    """输出单个矩阵的 Latex、SymPy 和代码三种形式。"""
    parts = [
        f"{name}",
        "-" * len(name),
        "Latex:",
        sp.latex(matrix),
        "",
        "SymPy:",
        str(matrix),
        "",
        "Code:",
        matrix_to_code(name, matrix),
        "",
    ]
    return "\n".join(parts)


def build_closure_matrix(a_matrices: List[sp.Matrix]) -> sp.Matrix:
    """顺序相乘得到完整闭环矩阵 T。"""
    closure = sp.eye(4)
    for matrix in a_matrices:
        closure = closure * matrix
    return closure


def zero_configuration_substitution() -> Dict[sp.Symbol, sp.Expr]:
    """零位替换：x=y=z=s=0。"""
    return {x: 0, y: 0, z: 0, s: 0}


def frobenius_norm(matrix: sp.Matrix) -> sp.Expr:
    """显式计算 Frobenius 范数，避免依赖额外数值库。"""
    return sp.sqrt(sum(entry**2 for entry in matrix))


def half_angle_substitution(expression: sp.Expr) -> sp.Expr:
    """
    对表达式做半角代换：
    X=tan(x/2), Y=tan(y/2), Z=tan(z/2), q=tan(s/2)
    这里只做代换，不做消元。
    """
    expanded = sp.expand_trig(expression)

    substitution_rules = {
        sp.sin(x): 2 * X / (1 + X**2),
        sp.cos(x): (1 - X**2) / (1 + X**2),
        sp.sin(y): 2 * Y / (1 + Y**2),
        sp.cos(y): (1 - Y**2) / (1 + Y**2),
        sp.sin(z): 2 * Z / (1 + Z**2),
        sp.cos(z): (1 - Z**2) / (1 + Z**2),
        sp.sin(s): 2 * q / (1 + q**2),
        sp.cos(s): (1 - q**2) / (1 + q**2),
    }

    replaced = expanded.subs(substitution_rules)
    return sp.together(replaced)


def total_degree_of_expression(expression: sp.Expr, variables: Iterable[sp.Symbol]) -> int:
    """计算多项式总次数；如果表达式为常数，则返回 0。"""
    variable_list = [symbol for symbol in variables if expression.has(symbol)]
    if not variable_list:
        return 0
    polynomial = sp.Poly(sp.expand(expression), *variable_list)
    return int(polynomial.total_degree())


def variable_presence_map(expression: sp.Expr) -> Dict[str, bool]:
    """记录 X、Y、Z、q 是否出现在表达式中。"""
    return {
        "X": bool(expression.has(X)),
        "Y": bool(expression.has(Y)),
        "Z": bool(expression.has(Z)),
        "q": bool(expression.has(q)),
    }


def build_equation_reports(left_matrix: sp.Matrix, right_matrix: sp.Matrix) -> List[EquationReport]:
    """为 16 个矩阵元素分别构造原三角表达式与半角代换结果。"""
    reports: List[EquationReport] = []

    for row_index in range(4):
        for col_index in range(4):
            name = f"E{row_index + 1}{col_index + 1}"
            trig_expression = left_matrix[row_index, col_index] - right_matrix[row_index, col_index]
            half_angle_expression = half_angle_substitution(trig_expression)
            numerator, denominator = sp.fraction(half_angle_expression)
            factored_numerator = sp.factor(numerator)
            total_degree = total_degree_of_expression(numerator, [X, Y, Z, q])

            reports.append(
                EquationReport(
                    name=name,
                    trig_expression=trig_expression,
                    half_angle_expression=half_angle_expression,
                    numerator=numerator,
                    denominator=denominator,
                    factored_numerator=factored_numerator,
                    total_degree=total_degree,
                    variable_presence=variable_presence_map(half_angle_expression),
                )
            )

    return reports


def write_text(path: Path, content: str) -> None:
    """统一的文本输出函数，确保目录存在并使用 UTF-8 编码。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")


def build_mdh_matrices_text(a_matrices: List[sp.Matrix]) -> str:
    """整理 A1~A7 的完整文本输出。"""
    sections = []
    for index, matrix in enumerate(a_matrices, start=1):
        sections.append(matrix_block_text(f"A{index}", matrix))
    return "\n".join(sections)


def build_closure_text(closure_matrix: sp.Matrix, closure_error_fro: sp.Expr) -> str:
    """整理完整闭环矩阵与零位误差结果。"""
    identity_matrix = sp.eye(4)
    residual_matrix = closure_matrix - identity_matrix
    zero_residual_matrix = residual_matrix.subs(zero_configuration_substitution())

    parts = [
        "T = A1*A2*A3*A4*A5*A6*A7",
        "------------------------",
        "Latex:",
        sp.latex(closure_matrix),
        "",
        "SymPy:",
        str(closure_matrix),
        "",
        "T - I at x=y=z=s=0:",
        str(zero_residual_matrix),
        "",
        "||T - I||_F at x=y=z=s=0:",
        str(closure_error_fro),
        "",
        "Numeric value:",
        str(sp.N(closure_error_fro, 20)),
        "",
    ]
    return "\n".join(parts)


def build_block_equations_text(left_matrix: sp.Matrix, right_matrix: sp.Matrix, reports: List[EquationReport]) -> str:
    """整理 k=4 分块方程文本。"""
    parts = [
        "L = A1*A2*A3*A4",
        "-------------",
        "Latex:",
        sp.latex(left_matrix),
        "",
        "SymPy:",
        str(left_matrix),
        "",
        "R = (A5*A6*A7)^(-1)",
        "-------------------",
        "Latex:",
        sp.latex(right_matrix),
        "",
        "SymPy:",
        str(right_matrix),
        "",
        "Element-wise equations",
        "----------------------",
    ]

    for report in reports:
        row_index = int(report.name[1]) - 1
        col_index = int(report.name[2]) - 1
        parts.extend(
            [
                f"{report.name}:",
                f"L{row_index + 1}{col_index + 1} = {left_matrix[row_index, col_index]}",
                f"R{row_index + 1}{col_index + 1} = {right_matrix[row_index, col_index]}",
                f"{report.name} = L{row_index + 1}{col_index + 1} - R{row_index + 1}{col_index + 1} = {report.trig_expression}",
                "",
            ]
        )

    return "\n".join(parts)


def build_half_angle_text(reports: List[EquationReport]) -> str:
    """整理 16 个方程的半角代换结果。"""
    parts: List[str] = []
    for report in reports:
        presence_text = ", ".join(f"{name}={value}" for name, value in report.variable_presence.items())
        parts.extend(
            [
                report.name,
                "-" * len(report.name),
                "1. 原三角表达式",
                str(report.trig_expression),
                "",
                "2. 半角表达式",
                str(report.half_angle_expression),
                "",
                "3. 分子",
                str(report.numerator),
                "",
                "4. 分母",
                str(report.denominator),
                "",
                "5. factor(分子)",
                str(report.factored_numerator),
                "",
                "6. 总次数",
                str(report.total_degree),
                "",
                "7. 变量出现情况",
                presence_text,
                "",
            ]
        )
    return "\n".join(parts)


def build_summary_text(reports: List[EquationReport], closure_error_fro: sp.Expr) -> str:
    """生成紧凑总表，便于后续人工挑选方程。"""
    header = "Name | Degree | HasX | HasY | HasZ | Hasq"
    separator = "---- | ------ | ---- | ---- | ---- | ----"
    rows = [header, separator]

    for report in reports:
        rows.append(
            " | ".join(
                [
                    report.name,
                    str(report.total_degree),
                    str(report.variable_presence["X"]),
                    str(report.variable_presence["Y"]),
                    str(report.variable_presence["Z"]),
                    str(report.variable_presence["q"]),
                ]
            )
        )

    rows.extend(
        [
            "",
            f"Closure error ||T-I||_F at x=y=z=s=0: {closure_error_fro}",
            f"Numeric closure error: {sp.N(closure_error_fro, 20)}",
            "",
            "Stop here by design: no elimination, no Groebner, no Resultant, no RootOf, no branch analysis.",
        ]
    )
    return "\n".join(rows)


def build_readme_text() -> str:
    """生成项目说明文件。"""
    return "\n".join(
        [
            "# symbolic_mdh_joint2",
            "",
            "本目录只做 Joint2 输入情形下的 Modified DH 闭环方程建立。",
            "",
            "固定约束：",
            "- 关节编号始终保持 Joint1~Joint7。",
            "- 输入变量为 s = Δθ2。",
            "- 未知量为 x = Δθ1, y = Δθ3, z = Δθ4。",
            "- 角度表达式固定为 θ1=x, θ2=s, θ3=π+y, θ4=z, θ5=-y, θ6=π-s, θ7=x。",
            "- 只输出 MDH、闭环、k=4 分块和半角代换结果。",
            "- 禁止消元、Groebner、Resultant、RootOf、Branch Analysis、Fourier、SVD、数值拟合。",
            "",
            "文件说明：",
            "- mdh_matrices.txt：A1~A7 的 Latex / SymPy / 代码表示。",
            "- closure_matrix.txt：完整闭环矩阵 T 及零位误差。",
            "- block_equations.txt：L=A1A2A3A4 与 R=(A5A6A7)^(-1) 的 16 个元素方程。",
            "- half_angle_equations.txt：16 个元素方程的半角代换、分子分母、factor 和总次数。",
            "- equation_summary.txt：16 个方程的变量出现情况与次数总览。",
            "- derive_joint2_mdh.py：本阶段唯一推导脚本。",
            "",
            "运行方式：",
            "```bash",
            "python symbolic_mdh_joint2/derive_joint2_mdh.py",
            "```",
        ]
    )


def build_all_outputs(output_dir: Path | str) -> Dict[str, object]:
    """生成本阶段所需的全部符号结果与文本文件。"""
    output_path = Path(output_dir)
    a_matrices = build_joint_matrices()
    closure_matrix = build_closure_matrix(a_matrices)
    closure_residual_zero = (closure_matrix - sp.eye(4)).subs(zero_configuration_substitution())
    closure_error_fro = sp.N(frobenius_norm(closure_residual_zero), 30)

    left_matrix = a_matrices[0] * a_matrices[1] * a_matrices[2] * a_matrices[3]
    right_matrix = (a_matrices[4] * a_matrices[5] * a_matrices[6]).inv()
    equation_reports = build_equation_reports(left_matrix, right_matrix)

    write_text(output_path / "mdh_matrices.txt", build_mdh_matrices_text(a_matrices))
    write_text(output_path / "closure_matrix.txt", build_closure_text(closure_matrix, closure_error_fro))
    write_text(output_path / "block_equations.txt", build_block_equations_text(left_matrix, right_matrix, equation_reports))
    write_text(output_path / "half_angle_equations.txt", build_half_angle_text(equation_reports))
    write_text(output_path / "equation_summary.txt", build_summary_text(equation_reports, closure_error_fro))
    write_text(output_path / "README.md", build_readme_text())

    return {
        "A_matrices": a_matrices,
        "T": closure_matrix,
        "L": left_matrix,
        "R": right_matrix,
        "equations": equation_reports,
        "closure_error_fro": closure_error_fro,
    }


def main() -> None:
    """命令行入口：默认把结果写入仓库根目录下的 symbolic_mdh_joint2。"""
    default_output_dir = Path(__file__).resolve().parent
    result = build_all_outputs(default_output_dir)
    print(f"Generated outputs in: {default_output_dir}")
    print(f"Closure error ||T-I||_F at zero configuration: {result['closure_error_fro']}")


if __name__ == "__main__":
    main()
