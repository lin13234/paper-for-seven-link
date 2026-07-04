"""
Symbolic elimination for the 7R closed-chain MDH model.

This script:
1. Builds the full MDH loop-closure matrix.
2. Extracts unique numerator polynomials after half-angle substitution.
3. Finds rank-3 independent equation triplets.
4. Prioritizes low-complexity triplets for resultant elimination.
5. Records elimination results and main-branch factor candidates.
"""

import itertools
import time
import traceback
from concurrent.futures import ProcessPoolExecutor, TimeoutError
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import sympy as sp


OUTPUT_DIR = Path(__file__).resolve().parent
EQUATIONS_FILE = OUTPUT_DIR / "equations_unique.txt"
INDEPENDENT_SETS_FILE = OUTPUT_DIR / "independent_sets.txt"
ELIMINATION_FILE = OUTPUT_DIR / "elimination_results.txt"
MAIN_BRANCH_FILE = OUTPUT_DIR / "main_branch_candidates.txt"

l = sp.symbols("l", nonzero=True)
s = sp.symbols("s", real=True)
x, y, z = sp.symbols("x y z", real=True)
q, X, Y, Z = sp.symbols("q X Y Z", real=True)

HALF_ANGLE_SYMBOLS = (X, Y, Z, q, l)
INDEX_OF = {X: 0, Y: 1, Z: 2, q: 3, l: 4}
JACOBIAN_POINT = {X: sp.Rational(1, 5), Y: sp.Rational(1, 7), Z: sp.Rational(1, 9), q: sp.Rational(1, 11), l: 1}
RESULTANT_TIMEOUT_SECONDS = 60
MAX_ELIMINATION_GROUPS = 12


@dataclass(frozen=True)
class EquationInfo:
    index: int
    polynomial: sp.Expr
    term_count: int
    total_degree: int
    degree_xyz: int


@dataclass(frozen=True)
class IndependentSetInfo:
    indices: tuple[int, int, int]
    score: tuple[int, int, int]
    determinant_sample: sp.Expr


def log(message: str) -> None:
    now = time.strftime("%H:%M:%S")
    print(f"[{now}] {message}", flush=True)


def rx(alpha: sp.Expr) -> sp.Matrix:
    ca = sp.cos(alpha)
    sa = sp.sin(alpha)
    return sp.Matrix([
        [1, 0, 0, 0],
        [0, ca, -sa, 0],
        [0, sa, ca, 0],
        [0, 0, 0, 1],
    ])


def tx(a_value: sp.Expr) -> sp.Matrix:
    return sp.Matrix([
        [1, 0, 0, a_value],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
    ])


def rz(theta: sp.Expr) -> sp.Matrix:
    ct = sp.cos(theta)
    st = sp.sin(theta)
    return sp.Matrix([
        [ct, -st, 0, 0],
        [st, ct, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
    ])


def tz(d_value: sp.Expr) -> sp.Matrix:
    return sp.Matrix([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, d_value],
        [0, 0, 0, 1],
    ])


def mdh_matrix(alpha_prev: sp.Expr, a_prev: sp.Expr, d_i: sp.Expr, theta_i: sp.Expr) -> sp.Matrix:
    return sp.simplify(rx(alpha_prev) * tx(a_prev) * rz(theta_i) * tz(d_i))


def build_symbolic_chain() -> tuple[list[sp.Matrix], sp.Matrix]:
    parameters = [
        (0, l, 0, x),
        (sp.pi / 2, 2 * l, sp.Rational(5, 2) * l, y),
        (sp.pi / 2, 0, 0, sp.pi + s),
        (sp.pi / 2, sp.Rational(5, 2) * l, 0, z),
        (sp.pi / 2, sp.Rational(5, 2) * l, 0, -s),
        (sp.pi / 2, 0, sp.Rational(5, 2) * l, sp.pi - y),
        (sp.pi / 2, 2 * l, 0, x),
    ]
    transforms = [mdh_matrix(alpha_i, a_i, d_i, theta_i) for alpha_i, a_i, d_i, theta_i in parameters]
    total = sp.eye(4)
    for transform in transforms:
        total = sp.simplify(total * transform)
    return transforms, sp.simplify(total)


def half_angle_substitutions() -> dict[sp.Expr, sp.Expr]:
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


def normalize_polynomial(expr: sp.Expr) -> sp.Expr:
    if expr == 0:
        return sp.Integer(0)
    expanded = sp.expand(sp.together(expr))
    poly = sp.Poly(expanded, *HALF_ANGLE_SYMBOLS, domain="QQ")
    normalized_expr = poly.as_expr()

    _, primitive = sp.primitive(normalized_expr)
    normalized_expr = sp.expand(primitive)

    poly = sp.Poly(normalized_expr, *HALF_ANGLE_SYMBOLS, domain="QQ")
    min_l_power = min((monom[INDEX_OF[l]] for monom, _ in poly.terms()), default=0)
    if min_l_power > 0:
        normalized_expr = sp.expand(normalized_expr / (l**min_l_power))

    poly = sp.Poly(normalized_expr, *HALF_ANGLE_SYMBOLS, domain="QQ")
    coeffs = poly.coeffs()
    if coeffs and coeffs[0] < 0:
        normalized_expr = -normalized_expr
    return sp.expand(normalized_expr)


def polynomial_complexity(expr: sp.Expr) -> tuple[int, int, int]:
    poly = sp.Poly(expr, *HALF_ANGLE_SYMBOLS, domain="QQ")
    term_count = len(poly.terms())
    total_degree = poly.total_degree()
    degree_xyz = max((monom[0] + monom[1] + monom[2] for monom, _ in poly.terms()), default=0)
    return term_count, total_degree, degree_xyz


def extract_unique_closure_polynomials(closure_matrix: sp.Matrix) -> list[sp.Expr]:
    substitutions = half_angle_substitutions()
    unique_map: dict[str, sp.Expr] = {}

    for row in range(closure_matrix.rows):
        for col in range(closure_matrix.cols):
            entry = sp.expand_trig(closure_matrix[row, col])
            if entry == 0:
                continue

            rational_entry = sp.cancel(sp.together(entry.subs(substitutions)))
            numerator, _ = sp.fraction(rational_entry)
            if numerator == 0:
                continue

            normalized = normalize_polynomial(numerator)
            if normalized == 0:
                continue

            key = sp.srepr(normalized)
            unique_map.setdefault(key, normalized)

    return list(unique_map.values())


def build_equation_infos(polynomials: Iterable[sp.Expr]) -> list[EquationInfo]:
    infos: list[EquationInfo] = []
    for index, polynomial in enumerate(polynomials, start=1):
        term_count, total_degree, degree_xyz = polynomial_complexity(polynomial)
        infos.append(EquationInfo(index=index, polynomial=polynomial, term_count=term_count, total_degree=total_degree, degree_xyz=degree_xyz))
    return infos


def save_equations(equations: list[EquationInfo]) -> None:
    lines = [
        "# Unique closure numerator polynomials after half-angle substitution",
        f"# Count = {len(equations)}",
        "",
    ]
    for info in equations:
        lines.append(f"[Eq {info.index}] terms={info.term_count}, total_degree={info.total_degree}, degree_xyz={info.degree_xyz}")
        lines.append(sp.sstr(sp.factor(info.polynomial)))
        lines.append("")
    EQUATIONS_FILE.write_text("\n".join(lines), encoding="utf-8")


def find_independent_equation_sets(equations: list[EquationInfo]) -> list[IndependentSetInfo]:
    log(f"Finding independent equation triplets from {len(equations)} equations")
    independent_sets: list[IndependentSetInfo] = []
    variables = [X, Y, Z]

    for combo in itertools.combinations(equations, 3):
        expressions = [item.polynomial.subs(l, 1) for item in combo]
        jacobian = sp.Matrix(expressions).jacobian(variables)
        jacobian_value = jacobian.subs(JACOBIAN_POINT)
        rank = jacobian_value.rank()
        if rank == 3:
            score = (
                sum(item.term_count for item in combo),
                sum(item.total_degree for item in combo),
                sum(item.degree_xyz for item in combo),
            )
            determinant_sample = sp.simplify(jacobian_value.det())
            independent_sets.append(IndependentSetInfo(indices=tuple(item.index for item in combo), score=score, determinant_sample=determinant_sample))

    independent_sets.sort(key=lambda item: (item.score, item.indices))
    return independent_sets


def save_independent_sets(independent_sets: list[IndependentSetInfo]) -> None:
    lines = [
        "# Rank-3 independent equation sets",
        f"# Count = {len(independent_sets)}",
        "# Sorted by (sum_terms, sum_total_degree, sum_degree_xyz)",
        "",
    ]
    for rank_index, item in enumerate(independent_sets, start=1):
        lines.append(f"[Set {rank_index}] eq_indices={item.indices}, score={item.score}, det_at_sample={sp.sstr(item.determinant_sample)}")
    INDEPENDENT_SETS_FILE.write_text("\n".join(lines), encoding="utf-8")


def elimination_worker(payload: tuple[tuple[int, int, int], tuple[sp.Expr, sp.Expr, sp.Expr]]) -> dict[str, object]:
    indices, expressions = payload
    eq1, eq2, eq3 = [sp.expand(expr.subs(l, 1)) for expr in expressions]

    result: dict[str, object] = {
        "indices": indices,
        "status": "ok",
        "g_yq": None,
        "factor_list": None,
        "main_branch_factors": [],
        "details": "",
    }

    try:
        res12_raw = sp.resultant(eq1, eq2, Z)
        res13_raw = sp.resultant(eq1, eq3, Z)

        if res12_raw == 0 and res13_raw == 0:
            raise ValueError("Both resultants after eliminating Z are zero")
        if res12_raw == 0:
            raise ValueError("resultant(eq1, eq2, Z) is zero")
        if res13_raw == 0:
            raise ValueError("resultant(eq1, eq3, Z) is zero")

        res12 = normalize_polynomial(res12_raw)
        res13 = normalize_polynomial(res13_raw)

        g_yq_raw = sp.resultant(res12, res13, X)
        if g_yq_raw == 0:
            raise ValueError("resultant after eliminating X is zero")
        g_yq = normalize_polynomial(g_yq_raw)

        factors = sp.factor_list(g_yq)
        main_branch_factors: list[str] = []
        for factor_expr, multiplicity in factors[1]:
            value_at_origin = sp.simplify(factor_expr.subs({Y: 0, q: 0}))
            if value_at_origin == 0:
                main_branch_factors.append(f"multiplicity={multiplicity}, factor={sp.sstr(factor_expr)}")

        result["g_yq"] = sp.sstr(g_yq)
        result["factor_list"] = sp.sstr(factors)
        result["main_branch_factors"] = main_branch_factors
        result["details"] = (
            f"resultant(eq{indices[0]}, eq{indices[1]}; Z) = {sp.sstr(res12)}\n"
            f"resultant(eq{indices[0]}, eq{indices[2]}; Z) = {sp.sstr(res13)}"
        )
        return result
    except Exception as exc:
        result["status"] = "failed"
        result["details"] = f"{type(exc).__name__}: {exc}\n{traceback.format_exc()}"
        return result
def prioritize_for_elimination(independent_sets: list[IndependentSetInfo], equation_map: dict[int, EquationInfo], limit: int = MAX_ELIMINATION_GROUPS) -> list[IndependentSetInfo]:
    sorted_sets = sorted(
        independent_sets,
        key=lambda item: (
            item.score,
            tuple((equation_map[idx].term_count, equation_map[idx].total_degree, equation_map[idx].degree_xyz, idx) for idx in item.indices),
            item.indices,
        ),
    )
    return sorted_sets[:limit]


def run_elimination(prioritized_sets: list[IndependentSetInfo], equation_map: dict[int, EquationInfo]) -> tuple[list[dict[str, object]], list[tuple[tuple[int, int, int], str]]]:
    results: list[dict[str, object]] = []
    skipped: list[tuple[tuple[int, int, int], str]] = []

    for index, item in enumerate(prioritized_sets, start=1):
        log(f"Elimination {index}/{len(prioritized_sets)} on equation set {item.indices} with score {item.score}")
        expressions = tuple(equation_map[eq_index].polynomial for eq_index in item.indices)
        payload = (item.indices, expressions)

        with ProcessPoolExecutor(max_workers=1) as executor:
            future = executor.submit(elimination_worker, payload)
            try:
                result = future.result(timeout=RESULTANT_TIMEOUT_SECONDS)
                results.append(result)
                log(f"Finished equation set {item.indices} with status {result['status']}")
            except TimeoutError:
                future.cancel()
                reason = f"resultant timeout after {RESULTANT_TIMEOUT_SECONDS} seconds"
                skipped.append((item.indices, reason))
                log(f"Skipped equation set {item.indices}: {reason}")
            except Exception as exc:
                reason = f"{type(exc).__name__}: {exc}"
                skipped.append((item.indices, reason))
                log(f"Skipped equation set {item.indices}: {reason}")

    return results, skipped


def save_elimination_results(all_independent_sets: list[IndependentSetInfo], prioritized_sets: list[IndependentSetInfo], results: list[dict[str, object]], skipped: list[tuple[tuple[int, int, int], str]]) -> None:
    prioritized_index_set = {item.indices for item in prioritized_sets}
    processed_index_set = {tuple(result["indices"]) for result in results}
    skipped_index_set = {indices for indices, _ in skipped}

    lines = [
        "# Elimination results",
        f"# Total rank-3 sets = {len(all_independent_sets)}",
        f"# Prioritized for resultant = {len(prioritized_sets)}",
        f"# Resultant timeout limit = {RESULTANT_TIMEOUT_SECONDS} seconds",
        "",
        "## Processed Sets",
        "",
    ]

    for result in results:
        lines.append(f"[Set {result['indices']}] status={result['status']}")
        lines.append(str(result["details"]))
        if result["g_yq"] is not None:
            lines.append(f"G(Y,q) = {result['g_yq']}")
        if result["factor_list"] is not None:
            lines.append(f"factor_list = {result['factor_list']}")
        if result["main_branch_factors"]:
            lines.append("main_branch_factors:")
            for factor in result["main_branch_factors"]:
                lines.append(f"  - {factor}")
        lines.append("")

    lines.append("## Skipped Sets")
    lines.append("")
    for indices, reason in skipped:
        lines.append(f"[Set {indices}] skipped_reason={reason}")
    lines.append("")

    lines.append("## Not Prioritized For Resultant")
    lines.append("")
    for item in all_independent_sets:
        if item.indices not in prioritized_index_set and item.indices not in processed_index_set and item.indices not in skipped_index_set:
            lines.append(f"[Set {item.indices}] skipped_reason=not prioritized due to higher complexity score {item.score}")

    ELIMINATION_FILE.write_text("\n".join(lines), encoding="utf-8")


def save_main_branch_candidates(results: list[dict[str, object]]) -> None:
    lines = [
        "# Main branch candidate factors satisfying factor(Y=0, q=0) = 0",
        "",
    ]
    found = False
    for result in results:
        if result["main_branch_factors"]:
            found = True
            lines.append(f"[Set {result['indices']}]")
            for factor in result["main_branch_factors"]:
                lines.append(factor)
            lines.append("")
    if not found:
        lines.append("No main branch candidate factor was found among successfully eliminated sets.")
    MAIN_BRANCH_FILE.write_text("\n".join(lines), encoding="utf-8")


def write_summary_readme() -> None:
    readme_path = OUTPUT_DIR / "README.md"
    text = f'''# symbolic_mdh_branch_solution

## MDH formula

This directory uses the modified DH convention:

```text
A_i = Rx(alpha_{{i-1}}) * Tx(a_{{i-1}}) * Rz(theta_i) * Tz(d_i)
```

## Symmetry relations

```text
Delta theta1 = Delta theta7
Delta theta2 = -Delta theta6
Delta theta5 = -Delta theta3
```

Define:

```text
s = Delta theta3
x = Delta theta1
y = Delta theta2
z = Delta theta4
```

Therefore:

```text
theta1 = x
theta2 = y
theta3 = pi + s
theta4 = z
theta5 = -s
theta6 = pi - y
theta7 = x
```

## Run symbolic elimination

From the repository root:

```powershell
python symbolic_mdh_branch_solution/derive_mdh_symbolic.py
```

Outputs:

- `equations_unique.txt`
- `independent_sets.txt`
- `elimination_results.txt`
- `main_branch_candidates.txt`

Main-branch candidate factors are saved in:

```text
symbolic_mdh_branch_solution/main_branch_candidates.txt
```

## Run numeric verification

From the repository root:

```powershell
python symbolic_mdh_branch_solution/verify_numeric.py
```

Outputs:

- `joint_solution_numeric.csv`
- `joint_angles_vs_delta_theta3.png`

## Notes

- Independent triplets are filtered by Jacobian rank.
- Resultant elimination is only attempted on low-complexity triplets first.
- Any triplet that exceeds {RESULTANT_TIMEOUT_SECONDS} seconds is skipped and recorded.
- Numeric verification starts with the full matrix residual and falls back to translation plus vee((R-R^T)/2) when needed.
'''
    readme_path.write_text(text, encoding="utf-8")


def main() -> None:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    log("Building the 7 MDH transforms")
    transforms, total = build_symbolic_chain()
    log(f"Number of MDH transforms = {len(transforms)}")

    log("Building closure matrix F = T - I")
    closure = sp.simplify(total - sp.eye(4))

    log("Extracting unique closure polynomials after half-angle substitution")
    unique_polynomials = extract_unique_closure_polynomials(closure)
    equations = build_equation_infos(unique_polynomials)
    save_equations(equations)
    log(f"Saved {len(equations)} unique equations to {EQUATIONS_FILE.name}")

    independent_sets = find_independent_equation_sets(equations)
    save_independent_sets(independent_sets)
    log(f"Saved {len(independent_sets)} independent equation sets to {INDEPENDENT_SETS_FILE.name}")

    equation_map = {item.index: item for item in equations}
    prioritized_sets = prioritize_for_elimination(independent_sets, equation_map)
    log(f"Prioritized {len(prioritized_sets)} equation sets for resultant elimination")

    results, skipped = run_elimination(prioritized_sets, equation_map)
    save_elimination_results(independent_sets, prioritized_sets, results, skipped)
    save_main_branch_candidates(results)
    write_summary_readme()

    log(f"Saved elimination log to {ELIMINATION_FILE.name}")
    log(f"Saved main-branch candidates to {MAIN_BRANCH_FILE.name}")
    log("Symbolic elimination workflow finished")


if __name__ == "__main__":
    main()



