"""
导出 k=4 分块闭环方程。

本脚本只做以下事情：
1. 按 Modified DH 生成 A1...A7；
2. 构造左侧 L = A1*A2*A3*A4；
3. 构造右侧 R = inv(A7)*inv(A6)*inv(A5)；
4. 列出 16 个矩阵元素方程 Lij - Rij = 0；
5. 对这 16 个方程逐个做半角代换并导出统计信息；
6. 生成可人工检查的 txt/csv/tex/README 文件。

禁止事项：
- 不做 resultant
- 不做 Groebner
- 不做 solve / RootOf
- 不做自动消元
- 不做自动筛选独立方程
"""

from __future__ import annotations

import csv
import time
from dataclasses import dataclass
from pathlib import Path

import sympy as sp


PROJECT_DIR = Path(__file__).resolve().parent
DEFAULT_OUTPUT_DIR = PROJECT_DIR / "block_k4_equations"

l = sp.symbols("l", nonzero=True)
s = sp.symbols("s", real=True)
x, y, z = sp.symbols("x y z", real=True)
q, X, Y, Z = sp.symbols("q X Y Z", real=True)
HALF_ANGLE_SYMBOLS = (X, Y, Z, q)


@dataclass(frozen=True)
class EquationRecord:
    """保存单个矩阵元素方程的完整信息。"""

    label: str
    source: str
    trig_expr: sp.Expr
    is_identity: bool
    half_angle_expr: sp.Expr
    denominator: sp.Expr
    numerator: sp.Expr
    factored_numerator: sp.Expr
    total_degree: int
    term_count: int
    contains_x: bool
    contains_y: bool
    contains_z: bool
    contains_q: bool
    complexity_score: int


def log(message: str) -> None:
    """打印进度日志，避免长时间无输出。"""
    now = time.strftime("%H:%M:%S")
    print(f"[{now}] {message}", flush=True)


def rx(alpha: sp.Expr) -> sp.Matrix:
    """绕 x 轴旋转的齐次变换。"""
    ca = sp.cos(alpha)
    sa = sp.sin(alpha)
    return sp.Matrix([
        [1, 0, 0, 0],
        [0, ca, -sa, 0],
        [0, sa, ca, 0],
        [0, 0, 0, 1],
    ])


def tx(a_value: sp.Expr) -> sp.Matrix:
    """沿 x 轴平移的齐次变换。"""
    return sp.Matrix([
        [1, 0, 0, a_value],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
    ])


def rz(theta: sp.Expr) -> sp.Matrix:
    """绕 z 轴旋转的齐次变换。"""
    ct = sp.cos(theta)
    st = sp.sin(theta)
    return sp.Matrix([
        [ct, -st, 0, 0],
        [st, ct, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
    ])


def tz(d_value: sp.Expr) -> sp.Matrix:
    """沿 z 轴平移的齐次变换。"""
    return sp.Matrix([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, d_value],
        [0, 0, 0, 1],
    ])


def mdh_matrix(alpha_prev: sp.Expr, a_prev: sp.Expr, d_i: sp.Expr, theta_i: sp.Expr) -> sp.Matrix:
    """严格按 Modified DH 公式构造单个关节矩阵。"""
    return rx(alpha_prev) * tx(a_prev) * rz(theta_i) * tz(d_i)


def build_joint_matrices() -> list[sp.Matrix]:
    """按题目给定参数顺序生成 A1...A7。"""
    parameters = [
        (0, l, 0, x),
        (sp.pi / 2, 2 * l, sp.Rational(5, 2) * l, y),
        (sp.pi / 2, 0, 0, sp.pi + s),
        (sp.pi / 2, sp.Rational(5, 2) * l, 0, z),
        (sp.pi / 2, sp.Rational(5, 2) * l, 0, -s),
        (sp.pi / 2, 0, sp.Rational(5, 2) * l, sp.pi - y),
        (sp.pi / 2, 2 * l, 0, x),
    ]
    return [mdh_matrix(alpha_i, a_i, d_i, theta_i) for alpha_i, a_i, d_i, theta_i in parameters]


def simplify_entry(expr: sp.Expr) -> sp.Expr:
    """
    对单个矩阵元素做温和整理。
    这里只做 trig 展开和基础化简，避免引入半角或求解步骤。
    """
    return sp.simplify(sp.expand_trig(sp.expand_mul(expr)))


def build_block_matrices() -> tuple[sp.Matrix, sp.Matrix]:
    """构造分块闭环左右矩阵 L 与 R。"""
    matrices = build_joint_matrices()
    a1, a2, a3, a4, a5, a6, a7 = matrices
    log("Building L = A1*A2*A3*A4")
    left = a1 * a2 * a3 * a4
    log("Building R = inv(A7)*inv(A6)*inv(A5)")
    right = a7.inv() * a6.inv() * a5.inv()

    left = left.applyfunc(simplify_entry)
    right = right.applyfunc(simplify_entry)
    return left, right


def half_angle_substitutions() -> dict[sp.Expr, sp.Expr]:
    """半角代换字典，仅用于方程展示与统计。"""
    return {
        sp.sin(x): 2 * X / (1 + X**2),
        sp.cos(x): (1 - X**2) / (1 + X**2),
        sp.sin(y): 2 * Y / (1 + Y**2),
        sp.cos(y): (1 - Y**2) / (1 + Y**2),
        sp.sin(z): 2 * Z / (1 + Z**2),
        sp.cos(z): (1 - Z**2) / (1 + Z**2),
        sp.sin(s): 2 * q / (1 + q**2),
        sp.cos(s): (1 - q**2) / (1 + q**2),
    }


def complexity_score_from_poly(poly: sp.Poly) -> int:
    """给统计表一个透明的复杂度分数，只用于排序，不做选优。"""
    return 100 * poly.total_degree() + len(poly.terms())


def build_equation_records(left: sp.Matrix, right: sp.Matrix) -> list[EquationRecord]:
    """构造 16 个矩阵元素方程及其半角统计信息。"""
    substitutions = half_angle_substitutions()
    records: list[EquationRecord] = []

    for row in range(4):
        for col in range(4):
            label = f"E{row + 1}{col + 1}"
            source = f"L{row + 1}{col + 1} = R{row + 1}{col + 1}"
            trig_expr = simplify_entry(left[row, col] - right[row, col])

            half_angle_expr = sp.cancel(sp.together(sp.expand_trig(trig_expr).subs(substitutions)))
            numerator, denominator = sp.fraction(half_angle_expr)
            is_identity = (trig_expr == 0) or (sp.simplify(trig_expr) == 0) or (sp.expand(numerator) == 0)
            numerator = sp.expand(numerator)
            denominator = sp.expand(denominator)

            if numerator == 0:
                total_degree = 0
                term_count = 0
                factored_numerator = sp.Integer(0)
                contains_x = False
                contains_y = False
                contains_z = False
                contains_q = False
                complexity_score = 0
            else:
                # ??? l ??????????????????????
                # ?? EX ?????????????????? X/Y/Z/q ???????
                poly = sp.Poly(numerator, *HALF_ANGLE_SYMBOLS, domain=sp.EX)
                total_degree = poly.total_degree()
                term_count = len(poly.terms())
                factored_numerator = sp.factor(numerator)
                contains_x = poly.degree(X) > 0
                contains_y = poly.degree(Y) > 0
                contains_z = poly.degree(Z) > 0
                contains_q = poly.degree(q) > 0
                complexity_score = complexity_score_from_poly(poly)

            records.append(
                EquationRecord(
                    label=label,
                    source=source,
                    trig_expr=trig_expr,
                    is_identity=is_identity,
                    half_angle_expr=half_angle_expr,
                    denominator=denominator,
                    numerator=numerator,
                    factored_numerator=factored_numerator,
                    total_degree=total_degree,
                    term_count=term_count,
                    contains_x=contains_x,
                    contains_y=contains_y,
                    contains_z=contains_z,
                    contains_q=contains_q,
                    complexity_score=complexity_score,
                )
            )
    return records


def format_matrix_txt(name: str, matrix: sp.Matrix) -> str:
    """把矩阵按逐元素编号格式写成纯文本。"""
    lines = [f"{name} matrix", ""]
    for row in range(4):
        for col in range(4):
            lines.append(f"{name}{row + 1}{col + 1} =")
            lines.append(sp.sstr(matrix[row, col]))
            lines.append("")
    return "\n".join(lines)


def write_left_right_matrices(output_dir: Path, left: sp.Matrix, right: sp.Matrix) -> None:
    """写出左右矩阵 txt。"""
    (output_dir / "block_k4_left_matrix.txt").write_text(format_matrix_txt("L", left), encoding="utf-8")
    (output_dir / "block_k4_right_matrix.txt").write_text(format_matrix_txt("R", right), encoding="utf-8")


def write_trig_equations(output_dir: Path, left: sp.Matrix, right: sp.Matrix, records: list[EquationRecord]) -> None:
    """写出 16 个三角形式矩阵元素方程。"""
    lines = [
        "k=4 block closure equations in trigonometric form",
        "Definition: L = A1*A2*A3*A4, R = inv(A7)*inv(A6)*inv(A5)",
        "",
    ]
    for row in range(4):
        for col in range(4):
            record = records[row * 4 + col]
            lines.append(f"编号：{record.label}")
            lines.append(f"来源：L{row + 1}{col + 1} = R{row + 1}{col + 1}")
            lines.append(f"L{row + 1}{col + 1} =")
            lines.append(sp.sstr(left[row, col]))
            lines.append(f"R{row + 1}{col + 1} =")
            lines.append(sp.sstr(right[row, col]))
            if record.is_identity:
                lines.append("完整表达式：0 = 0")
                lines.append("说明：恒等成立")
            else:
                lines.append("完整表达式：")
                lines.append(f"{sp.sstr(record.trig_expr)} = 0")
                lines.append("说明：非恒等方程")
            lines.append("")
    (output_dir / "block_k4_equations_trig.txt").write_text("\n".join(lines), encoding="utf-8")


def write_half_angle_equations(output_dir: Path, records: list[EquationRecord]) -> None:
    """写出 16 个方程的半角形式与分子分母统计。"""
    lines = [
        "k=4 block closure equations after half-angle substitution",
        "",
    ]
    for record in records:
        lines.append(f"编号：{record.label}")
        lines.append(f"来源：{record.source}")
        lines.append("① 原三角表达式")
        lines.append(sp.sstr(record.trig_expr))
        lines.append("② 半角表达式")
        lines.append(sp.sstr(record.half_angle_expr))
        lines.append("③ 分母")
        lines.append(sp.sstr(record.denominator))
        lines.append("④ 分子")
        lines.append(sp.sstr(record.numerator))
        lines.append("⑤ 分子因式分解")
        lines.append(sp.sstr(record.factored_numerator))
        lines.append("⑥ 分子总次数")
        lines.append(str(record.total_degree))
        lines.append("⑦ 分子项数")
        lines.append(str(record.term_count))
        lines.append("⑧ 是否含 X/Y/Z/q")
        lines.append(f"X={record.contains_x}, Y={record.contains_y}, Z={record.contains_z}, q={record.contains_q}")
        if record.is_identity:
            lines.append("备注：恒等成立")
        lines.append("")
    (output_dir / "block_k4_equations_half_angle.txt").write_text("\n".join(lines), encoding="utf-8")


def write_summary_csv(output_dir: Path, records: list[EquationRecord]) -> None:
    """写出 16 个方程的统计表，按复杂度升序排序。"""
    sorted_records = sorted(records, key=lambda item: (item.complexity_score, item.total_degree, item.term_count, item.label))
    with (output_dir / "block_k4_equation_summary.csv").open("w", encoding="utf-8", newline="") as file:
        writer = csv.DictWriter(
            file,
            fieldnames=[
                "编号",
                "矩阵元素来源",
                "是否恒等",
                "分子次数",
                "项数",
                "是否含X",
                "是否含Y",
                "是否含Z",
                "是否含q",
                "复杂度评分",
            ],
        )
        writer.writeheader()
        for record in sorted_records:
            writer.writerow(
                {
                    "编号": record.label,
                    "矩阵元素来源": record.source,
                    "是否恒等": "是" if record.is_identity else "否",
                    "分子次数": record.total_degree,
                    "项数": record.term_count,
                    "是否含X": "是" if record.contains_x else "否",
                    "是否含Y": "是" if record.contains_y else "否",
                    "是否含Z": "是" if record.contains_z else "否",
                    "是否含q": "是" if record.contains_q else "否",
                    "复杂度评分": record.complexity_score,
                }
            )


def matrix_to_latex(name: str, matrix: sp.Matrix) -> str:
    """把矩阵转成论文可直接复制的 LaTeX。"""
    rows = []
    for row in range(4):
        row_entries = [sp.latex(matrix[row, col]) for col in range(4)]
        rows.append(" & ".join(row_entries))
    body = " \\\\\n".join(rows)
    return f"{name} = \\begin{{bmatrix}}\n{body}\n\\end{{bmatrix}}"


def write_latex_file(output_dir: Path, left: sp.Matrix, right: sp.Matrix, records: list[EquationRecord]) -> None:
    """生成 LaTeX 文件，完整列出 L、R 与 16 个方程。"""
    lines = [
        "% k=4 block closure equations",
        "\\section*{k=4 分块闭环方程}",
        "",
        "\\subsection*{左侧矩阵}",
        "\\[",
        matrix_to_latex("L", left),
        "\\]",
        "",
        "\\subsection*{右侧矩阵}",
        "\\[",
        matrix_to_latex("R", right),
        "\\]",
        "",
        "\\subsection*{16 个矩阵元素方程}",
    ]
    for record in records:
        lines.append(f"\\paragraph{{{record.label}}} 来源：${record.source}$.")
        lines.append("\\[")
        lines.append(f"{sp.latex(record.trig_expr)} = 0")
        lines.append("\\]")
        if record.is_identity:
            lines.append("该式恒等成立。")
        lines.append("")
    (output_dir / "block_k4_equations_latex.tex").write_text("\n".join(lines), encoding="utf-8")


def write_readme(output_dir: Path) -> None:
    """说明本目录每个文件的作用。"""
    text = """# block_k4_equations

这个目录只保存 `k=4` 分块闭环方程的显式列式结果，不包含任何自动求解或自动消元。

## 文件说明

- `block_k4_left_matrix.txt`
  保存左侧矩阵 `L = A1*A2*A3*A4` 的完整 4x4 元素表达式。
- `block_k4_right_matrix.txt`
  保存右侧矩阵 `R = inv(A7)*inv(A6)*inv(A5)` 的完整 4x4 元素表达式。
- `block_k4_equations_trig.txt`
  保存 16 个矩阵元素闭环方程 `Lij - Rij = 0` 的三角形式。
- `block_k4_equations_half_angle.txt`
  保存同一批 16 个方程在半角代换后的表达式、分子分母、因式分解和复杂度统计。
- `block_k4_equation_summary.csv`
  保存 16 个方程的统计表，并按复杂度评分排序。
- `block_k4_equations_latex.tex`
  保存适合直接复制到论文中的 LaTeX 版本。

## 哪个文件对应三角形式

- `block_k4_equations_trig.txt`

## 哪个文件对应半角形式

- `block_k4_equations_half_angle.txt`

## 哪个文件适合后续人工消元

- `block_k4_equations_trig.txt`
- `block_k4_equations_half_angle.txt`
- `block_k4_equation_summary.csv`

其中：
- 三角形式适合先做人工结构观察；
- 半角形式适合人工挑选低复杂度方程；
- CSV 适合先按复杂度和变量出现情况快速筛读。

## 后续工作原则

后续不再由脚本自动求解，而是由人工从这 16 个显式方程中选择合适的方程，开展解析推导。
"""
    (output_dir / "README.md").write_text(text, encoding="utf-8")


def export_block_k4_bundle(output_dir: Path = DEFAULT_OUTPUT_DIR) -> Path:
    """导出整套 k=4 分块闭环文件。"""
    output_dir.mkdir(parents=True, exist_ok=True)
    log(f"Output directory: {output_dir}")

    left, right = build_block_matrices()
    log("Building 16 block equations")
    records = build_equation_records(left, right)

    log("Writing matrix text files")
    write_left_right_matrices(output_dir, left, right)

    log("Writing trigonometric equation file")
    write_trig_equations(output_dir, left, right, records)

    log("Writing half-angle equation file")
    write_half_angle_equations(output_dir, records)

    log("Writing summary CSV")
    write_summary_csv(output_dir, records)

    log("Writing LaTeX file")
    write_latex_file(output_dir, left, right, records)

    log("Writing README")
    write_readme(output_dir)

    return output_dir


def main() -> None:
    """脚本入口。"""
    export_block_k4_bundle(DEFAULT_OUTPUT_DIR)
    log("k=4 block closure export finished")


if __name__ == "__main__":
    main()
