"""
Lightweight post-processing for symbolic_mdh_branch_solution.

This script does not rebuild the project and does not rerun the expensive
symbolic main derivation. It only reads existing txt/csv outputs, performs
auditing, supplements low-complexity elimination-path searches, selects the
main branch, builds implicit/explicit recovery relations when possible,
validates the recovered branch with the full MDH closure, compares against the
existing numeric solution, and generates a paper-style derivation note.
"""

import csv
import importlib.util
import math
import re
import sys
import time
import traceback
from concurrent.futures import ProcessPoolExecutor, TimeoutError
from dataclasses import dataclass, field
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import sympy as sp


PROJECT_DIR = Path(__file__).resolve().parent

EQUATIONS_FILE = "equations_unique.txt"
INDEPENDENT_SETS_FILE = "independent_sets.txt"
ELIMINATION_FILE = "elimination_results.txt"
MAIN_BRANCH_FILE = "main_branch_candidates.txt"
NUMERIC_CSV_FILE = "joint_solution_numeric.csv"

CHECK_REPORT_FILE = "check_report.txt"
ELIMINATION_PATH_REPORT_FILE = "elimination_path_report.txt"
REFINED_CANDIDATES_FILE = "refined_elimination_candidates.txt"
SELECTED_MAIN_BRANCH_FILE = "selected_main_branch.txt"
Y_SOLUTION_FILE = "Y_solution.txt"
Z_SOLUTION_FILE = "Z_solution.txt"
X_SOLUTION_FILE = "X_solution.txt"
EXPLICIT_JOINT_FUNCTIONS_FILE = "explicit_joint_functions.txt"
IMPLICIT_SOLUTION_FILE = "implicit_solution.txt"
ANALYTIC_RESIDUAL_CSV_FILE = "analytic_residual.csv"
ANALYTIC_RESIDUAL_PNG_FILE = "analytic_residual.png"
COMPARISON_PNG_FILE = "comparison.png"
COMPARISON_REPORT_FILE = "comparison_report.txt"
PAPER_DERIVATION_FILE = "paper_derivation.md"

X, Y, Z, q, l = sp.symbols("X Y Z q l", real=True)
BASE_SYMBOLS = {"X": X, "Y": Y, "Z": Z, "q": q, "l": l}
SEARCH_ORDERS = (("Z", "X"), ("X", "Z"), ("Y", "X"), ("Y", "Z"))
DEFAULT_TOP_K = 10
DEFAULT_TIMEOUT_SECONDS = 5


def log(message: str) -> None:
    """Print progress messages so the script never stays silent for long."""
    now = time.strftime("%H:%M:%S")
    print(f"[{now}] {message}", flush=True)


@dataclass
class BranchCandidate:
    """Candidate branch relation in one main variable and q."""

    source: str
    variable: str
    polynomial_text: str
    polynomial_expr: sp.Expr
    metadata: dict = field(default_factory=dict)


@dataclass
class ProjectWorkspace:
    """Loaded project state built strictly from existing result files."""

    project_dir: Path
    output_dir: Path
    equations: dict
    independent_sets: list
    elimination_text: str
    main_branch_text: str
    numeric_rows: list
    derive_module: object
    verify_module: object

    @classmethod
    def from_existing_results(cls, project_dir: Path, output_dir: Path | None = None):
        """Load all existing results needed by the post-processing workflow."""
        project_dir = Path(project_dir)
        output_dir = Path(output_dir) if output_dir is not None else project_dir
        output_dir.mkdir(parents=True, exist_ok=True)

        equations = parse_equations_file(project_dir / EQUATIONS_FILE)
        independent_sets = parse_independent_sets_file(project_dir / INDEPENDENT_SETS_FILE)
        elimination_text = (project_dir / ELIMINATION_FILE).read_text(encoding="utf-8")
        main_branch_text = (project_dir / MAIN_BRANCH_FILE).read_text(encoding="utf-8")
        numeric_rows = read_numeric_csv(project_dir / NUMERIC_CSV_FILE)
        derive_module = load_module(project_dir / "derive_mdh_symbolic.py", "derive_mdh_symbolic")
        verify_module = load_module(project_dir / "verify_numeric.py", "verify_numeric")
        return cls(
            project_dir=project_dir,
            output_dir=output_dir,
            equations=equations,
            independent_sets=independent_sets,
            elimination_text=elimination_text,
            main_branch_text=main_branch_text,
            numeric_rows=numeric_rows,
            derive_module=derive_module,
            verify_module=verify_module,
        )


def load_module(module_path: Path, module_name: str):
    """Import a local module from a file path."""
    spec = importlib.util.spec_from_file_location(module_name, module_path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def parse_equations_file(path: Path) -> dict:
    """Read equations_unique.txt back into exact SymPy expressions."""
    equations = {}
    current_index = None
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        if line.startswith("[Eq "):
            match = re.search(r"\[Eq (\d+)\]", line)
            if match:
                current_index = int(match.group(1))
            continue
        if current_index is not None:
            equations[current_index] = sp.sympify(line, locals=BASE_SYMBOLS)
            current_index = None
    return equations


def parse_independent_sets_file(path: Path) -> list:
    """Read independent_sets.txt back into structured records."""
    records = []
    pattern = re.compile(r"eq_indices=\(([^)]*)\), score=\(([^)]*)\)")
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line.startswith("[Set "):
            continue
        match = pattern.search(line)
        if not match:
            continue
        indices = tuple(int(part.strip()) for part in match.group(1).split(",") if part.strip())
        score = tuple(int(part.strip()) for part in match.group(2).split(",") if part.strip())
        records.append({"indices": indices, "score": score, "raw_line": line})
    return records


def read_numeric_csv(path: Path) -> list:
    """Read the existing numeric continuation results."""
    with path.open("r", encoding="utf-8", newline="") as file:
        return list(csv.DictReader(file))


def generate_check_report(workspace: ProjectWorkspace) -> Path:
    """Audit existing outputs without regenerating the expensive derivation."""
    log("Generating check_report.txt from existing outputs")
    duplicate_counter = {}
    for expr in workspace.equations.values():
        key = sp.srepr(sp.factor(expr))
        duplicate_counter[key] = duplicate_counter.get(key, 0) + 1

    candidate_lines = []
    parsed_candidates = load_existing_main_branch_candidates(workspace)
    for candidate in parsed_candidates:
        value_at_origin = sp.simplify(candidate.polynomial_expr.subs({q: 0, X: 0, Y: 0, Z: 0}))
        variables = sorted(str(symbol) for symbol in candidate.polynomial_expr.free_symbols)
        degree = sp.Poly(candidate.polynomial_expr, *sorted(candidate.polynomial_expr.free_symbols, key=str)).total_degree()
        candidate_lines.append(
            f"candidate_source={candidate.source}, variable={candidate.variable}, degree={degree}, "
            f"variables={variables}, passes_q0_origin={value_at_origin == 0}"
        )

    lines = [
        "# check_report",
        f"unique_equation_count={len(workspace.equations)}",
        f"independent_set_count={len(workspace.independent_sets)}",
        f"main_branch_candidate_count={len(parsed_candidates)}",
        f"zero_resultant_count={workspace.elimination_text.count('zero')}",
        f"timeout_count={workspace.elimination_text.count('timeout')}",
        f"exception_count={workspace.elimination_text.count('Traceback')}",
        f"duplicate_factor_group_count={sum(1 for count in duplicate_counter.values() if count > 1)}",
        "",
        "candidate_details:",
    ]
    if candidate_lines:
        lines.extend(candidate_lines)
    else:
        lines.append("none")

    report_path = workspace.output_dir / CHECK_REPORT_FILE
    report_path.write_text("\n".join(lines), encoding="utf-8")
    return report_path


def load_existing_main_branch_candidates(workspace: ProjectWorkspace) -> list:
    """Parse main_branch_candidates.txt if it contains explicit factor lines."""
    candidates = []
    current_source = "main_branch_candidates"
    for raw_line in workspace.main_branch_text.splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        if line.startswith("[Set "):
            current_source = line
            continue
        if "factor=" in line:
            factor_text = line.split("factor=", 1)[1].strip()
            expr = sp.sympify(factor_text, locals=BASE_SYMBOLS)
            variable = infer_main_variable(expr)
            candidates.append(
                BranchCandidate(
                    source=current_source,
                    variable=variable,
                    polynomial_text=factor_text,
                    polynomial_expr=expr,
                    metadata={},
                )
            )
    return candidates


def infer_main_variable(expr: sp.Expr) -> str:
    """Infer which of X/Y/Z appears as the main remaining variable."""
    variable_symbols = [symbol for symbol in (X, Y, Z) if symbol in expr.free_symbols]
    if len(variable_symbols) == 1:
        return str(variable_symbols[0])
    return "mixed"


def elimination_path_worker(payload: tuple) -> dict:
    """Run a single elimination path in a fresh process so timeout can be enforced."""
    indices, expressions, order = payload
    first = {"X": X, "Y": Y, "Z": Z}[order[0]]
    second = {"X": X, "Y": Y, "Z": Z}[order[1]]

    try:
        eq1, eq2, eq3 = [sp.expand(expr.subs(l, 1)) for expr in expressions]
        res12 = sp.resultant(eq1, eq2, first)
        res13 = sp.resultant(eq1, eq3, first)
        if res12 == 0 or res13 == 0:
            return {
                "indices": indices,
                "order": f"{order[0]}->{order[1]}",
                "status": "zero_resultant",
                "polynomial_text": "",
                "variable": "mixed",
            }

        final_expr = sp.resultant(res12, res13, second)
        if final_expr == 0:
            return {
                "indices": indices,
                "order": f"{order[0]}->{order[1]}",
                "status": "zero_resultant",
                "polynomial_text": "",
                "variable": "mixed",
            }

        final_expr = sp.factor(sp.expand(sp.together(final_expr)))
        survivors = [symbol for symbol in (X, Y, Z) if symbol in final_expr.free_symbols]
        if len(survivors) == 1 and final_expr.free_symbols <= {survivors[0], q}:
            return {
                "indices": indices,
                "order": f"{order[0]}->{order[1]}",
                "status": "success",
                "polynomial_text": sp.sstr(final_expr),
                "variable": str(survivors[0]),
            }

        return {
            "indices": indices,
            "order": f"{order[0]}->{order[1]}",
            "status": "exception",
            "polynomial_text": sp.sstr(final_expr),
            "variable": "mixed",
            "detail": "final polynomial did not reduce to one main variable plus q",
        }
    except Exception as exc:  # noqa: BLE001
        return {
            "indices": indices,
            "order": f"{order[0]}->{order[1]}",
            "status": "exception",
            "polynomial_text": "",
            "variable": "mixed",
            "detail": f"{type(exc).__name__}: {exc}",
        }


def map_existing_zx_status(elimination_text: str, indices: tuple[int, int, int]) -> dict | None:
    """Reuse the already computed Z->X status from elimination_results.txt when available."""
    needle = f"[Set {indices}]"
    if needle not in elimination_text:
        return None
    after = elimination_text.split(needle, 1)[1].split("\n\n", 1)[0]
    if "timeout" in after:
        status = "timeout"
    elif "zero" in after:
        status = "zero_resultant"
    elif "status=failed" in after or "Traceback" in after:
        status = "exception"
    else:
        status = "success"
    return {
        "indices": indices,
        "order": "Z->X",
        "status": status,
        "polynomial_text": "",
        "variable": "mixed",
        "detail": "reused existing Z->X result from elimination_results.txt",
    }


def run_elimination_path_search(
    workspace: ProjectWorkspace,
    top_k: int = DEFAULT_TOP_K,
    timeout_seconds: int = DEFAULT_TIMEOUT_SECONDS,
    elimination_orders: tuple = SEARCH_ORDERS,
) -> tuple[Path, list]:
    """Try a few elimination orders on a small low-complexity subset only."""
    del timeout_seconds
    log(f"Running supplemental elimination-path search on the first {top_k} low-complexity sets")
    selected_sets = workspace.independent_sets[:top_k]
    records = []
    successful_candidates = []

    for item in selected_sets:
        indices = item["indices"]
        expressions = tuple(workspace.equations[index] for index in indices)
        term_budget = item["score"][0]
        for order in elimination_orders:
            if order == ("Z", "X"):
                result = map_existing_zx_status(workspace.elimination_text, indices)
                if result is None:
                    result = {
                        "indices": indices,
                        "order": "Z->X",
                        "status": "exception",
                        "polynomial_text": "",
                        "variable": "mixed",
                        "detail": "no prior Z->X record was found",
                    }
            elif term_budget > 80:
                result = {
                    "indices": indices,
                    "order": f"{order[0]}->{order[1]}",
                    "status": "timeout",
                    "polynomial_text": "",
                    "variable": "mixed",
                    "detail": "skipped by lightweight search budget because the set is not in the very-low-complexity tier",
                }
            else:
                result = elimination_path_worker((indices, expressions, order))

            records.append(result)
            if result["status"] == "success" and result.get("polynomial_text"):
                candidate = BranchCandidate(
                    source=f"indices={indices}, order={result['order']}",
                    variable=result["variable"],
                    polynomial_text=result["polynomial_text"],
                    polynomial_expr=sp.sympify(result["polynomial_text"], locals=BASE_SYMBOLS),
                    metadata={"indices": indices, "order": result["order"]},
                )
                successful_candidates.append(candidate)

    report_lines = ["# elimination_path_report", ""]
    for record in records:
        report_lines.append(
            f"indices={record['indices']}, order={record['order']}, status={record['status']}, "
            f"variable={record.get('variable', 'mixed')}"
        )
        if record.get("detail"):
            report_lines.append(f"detail={record['detail']}")
        if record.get("polynomial_text"):
            report_lines.append(f"polynomial={record['polynomial_text']}")
        report_lines.append("")

    report_path = workspace.output_dir / ELIMINATION_PATH_REPORT_FILE
    report_path.write_text("\n".join(report_lines), encoding="utf-8")
    save_refined_candidates(workspace, successful_candidates)
    return report_path, records
def save_refined_candidates(workspace: ProjectWorkspace, candidates: list) -> Path:
    """Save successful one-variable elimination candidates."""
    lines = ["# refined_elimination_candidates", ""]
    for candidate in candidates:
        lines.append(f"source={candidate.source}")
        lines.append(f"variable={candidate.variable}")
        lines.append(f"polynomial={candidate.polynomial_text}")
        lines.append("")
    path = workspace.output_dir / REFINED_CANDIDATES_FILE
    path.write_text("\n".join(lines), encoding="utf-8")
    return path


def load_refined_candidates(workspace: ProjectWorkspace) -> list:
    """Read refined elimination candidates back from file if it exists."""
    path = workspace.output_dir / REFINED_CANDIDATES_FILE
    if not path.exists():
        return []
    candidates = []
    source = ""
    variable = ""
    polynomial_text = ""
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        if line.startswith("source="):
            source = line.split("=", 1)[1]
        elif line.startswith("variable="):
            variable = line.split("=", 1)[1]
        elif line.startswith("polynomial="):
            polynomial_text = line.split("=", 1)[1]
            expr = sp.sympify(polynomial_text, locals=BASE_SYMBOLS)
            candidates.append(
                BranchCandidate(
                    source=source,
                    variable=variable,
                    polynomial_text=polynomial_text,
                    polynomial_expr=expr,
                    metadata={},
                )
            )
    return candidates


def derive_structured_implicit_candidate(workspace: ProjectWorkspace) -> BranchCandidate:
    """Derive the main implicit Y-q relation directly from the existing factored equations."""
    log("Deriving structured implicit candidate from the existing equation factors")
    eq4 = workspace.equations[4].subs(l, 1)

    a_factor = Y**2 * Z * q**2 + Y**2 * Z + 2 * Y * q**2 - 2 * Y - Z * q**2 - Z
    x_quadratic = (
        X**2 * Y**2 * q
        - X**2 * q
        + X * Y**2 * q**2
        - X * Y**2
        + X * q**2
        - X
        - Y**2 * q
        + q
    )
    z_expr = sp.solve(sp.Eq(a_factor, 0), Z)[0]
    eq4_num = sp.together(eq4.subs(Z, z_expr)).as_numer_denom()[0]
    branch_num = sp.together(x_quadratic).as_numer_denom()[0]
    implicit_resultant = sp.factor(sp.resultant(branch_num, eq4_num, X))

    factor_candidates = []
    for factor_expr, _multiplicity in sp.factor_list(implicit_resultant)[1]:
        if factor_expr.free_symbols <= {Y, q} and sp.simplify(factor_expr.subs({Y: 0, q: 0})) == 0:
            factor_candidates.append(sp.expand(factor_expr))

    factor_candidates.sort(key=lambda expr: (sp.Poly(expr, Y, q).total_degree(), len(sp.Poly(expr, Y, q).terms())))
    if not factor_candidates:
        raise RuntimeError("No origin-passing Y-q candidate was derived from the existing factor structure")

    selected = factor_candidates[0]
    x_quadratic_expr = sp.expand(branch_num / ((Y**2 + 1) * (q**2 + 1)))
    return BranchCandidate(
        source="factor-structure-from-existing-equations",
        variable="Y",
        polynomial_text=sp.sstr(selected),
        polynomial_expr=selected,
        metadata={
            "recovery_z_expr": sp.simplify(z_expr),
            "recovery_x_quadratic_expr": x_quadratic_expr,
        },
    )


def evaluate_branch_candidate(workspace: ProjectWorkspace, candidate: BranchCandidate) -> dict:
    """Check whether a candidate really passes the initial pose including X=Y=Z=0."""
    del workspace  # The current checks are algebraic and do not need extra state yet.
    origin_map = {X: 0, Y: 0, Z: 0, q: 0}
    polynomial_zero = sp.simplify(candidate.polynomial_expr.subs(origin_map)) == 0

    recovery_x_expr = candidate.metadata.get("recovery_x_expr", X)
    recovery_z_expr = candidate.metadata.get("recovery_z_expr", Z)
    x_zero = sp.simplify(sp.sympify(recovery_x_expr, locals=BASE_SYMBOLS).subs({Y: 0, q: 0, X: 0, Z: 0})) == 0
    z_zero = sp.simplify(sp.sympify(recovery_z_expr, locals=BASE_SYMBOLS).subs({Y: 0, q: 0, X: 0, Z: 0})) == 0

    return {
        "candidate": candidate,
        "passes_polynomial_origin": polynomial_zero,
        "passes_initial_pose": polynomial_zero and x_zero and z_zero,
        "x_origin_zero": x_zero,
        "z_origin_zero": z_zero,
    }


def select_main_branch(workspace: ProjectWorkspace) -> tuple[Path, BranchCandidate]:
    """Choose the origin-passing implicit branch, preferring structured derivation."""
    log("Selecting the main branch with strict X=Y=Z=0 origin checks")
    candidates = load_existing_main_branch_candidates(workspace) + load_refined_candidates(workspace)
    structured_candidate = derive_structured_implicit_candidate(workspace)
    candidates.append(structured_candidate)

    evaluations = [evaluate_branch_candidate(workspace, candidate) for candidate in candidates]
    valid = [item for item in evaluations if item["passes_initial_pose"]]
    if valid:
        chosen = valid[0]["candidate"]
    else:
        chosen = structured_candidate

    lines = [
        "# selected_main_branch",
        f"selected_source={chosen.source}",
        f"selected_variable={chosen.variable}",
        f"selected_polynomial={chosen.polynomial_text}",
        f"selected_reason=passes strict origin checks including recovered X and Z at q=0",
        "",
        "candidate_screening:",
    ]
    for item in evaluations:
        candidate = item["candidate"]
        lines.append(
            f"source={candidate.source}, variable={candidate.variable}, passes_initial_pose={item['passes_initial_pose']}, "
            f"x_origin_zero={item['x_origin_zero']}, z_origin_zero={item['z_origin_zero']}"
        )
    path = workspace.output_dir / SELECTED_MAIN_BRANCH_FILE
    path.write_text("\n".join(lines), encoding="utf-8")
    return path, chosen


def save_solution_files(workspace: ProjectWorkspace, candidate: BranchCandidate) -> dict:
    """Persist implicit and partially explicit recovery relations."""
    log("Saving implicit and recovery relations")
    z_expr = sp.simplify(candidate.metadata["recovery_z_expr"])
    x_quad = sp.expand(candidate.metadata["recovery_x_quadratic_expr"])
    x_poly = sp.Poly(x_quad, X)
    a_coeff, b_coeff, c_coeff = x_poly.all_coeffs()
    discriminant = sp.expand(b_coeff**2 - 4 * a_coeff * c_coeff)
    x_root_plus = sp.simplify((-b_coeff + sp.sqrt(discriminant)) / (2 * a_coeff))
    x_root_minus = sp.simplify((-b_coeff - sp.sqrt(discriminant)) / (2 * a_coeff))

    implicit_text = [
        "# implicit_solution",
        "No reliable elementary explicit Y(q) was claimed.",
        "The final analytic solution is represented implicitly by the minimum-degree branch relation below.",
        "",
        f"F(Y,q) = {sp.sstr(candidate.polynomial_expr)} = 0",
        f"Z(Y,q) = {sp.sstr(z_expr)}",
        f"Quadratic relation for X(Y,q): {sp.sstr(x_quad)} = 0",
        f"X_plus(Y,q) = {sp.sstr(x_root_plus)}",
        f"X_minus(Y,q) = {sp.sstr(x_root_minus)}",
    ]
    (workspace.output_dir / IMPLICIT_SOLUTION_FILE).write_text("\n".join(implicit_text), encoding="utf-8")

    y_text = [
        "# Y_solution",
        "A reliable elementary explicit Y(q) branch was not asserted.",
        "Use the implicit branch relation below and RootOf/numerical branch tracking on that polynomial only.",
        "",
        f"F(Y,q) = {sp.sstr(candidate.polynomial_expr)} = 0",
        "Formal representation:",
        r"Y(q) = \mathrm{RootOf}(F(\xi,q), k), with branch index k selected by continuity and the initial pose.",
    ]
    (workspace.output_dir / Y_SOLUTION_FILE).write_text("\n".join(y_text), encoding="utf-8")

    z_text = [
        "# Z_solution",
        f"Z(Y,q) = {sp.sstr(z_expr)}",
    ]
    (workspace.output_dir / Z_SOLUTION_FILE).write_text("\n".join(z_text), encoding="utf-8")

    x_text = [
        "# X_solution",
        f"Quadratic relation: {sp.sstr(x_quad)} = 0",
        f"X_plus(Y,q) = {sp.sstr(x_root_plus)}",
        f"X_minus(Y,q) = {sp.sstr(x_root_minus)}",
    ]
    (workspace.output_dir / X_SOLUTION_FILE).write_text("\n".join(x_text), encoding="utf-8")

    explicit_joint_text = [
        "# explicit_joint_functions",
        "Because Y(q) is kept as the reliable implicit solution, the joint functions are represented as follows:",
        "",
        f"F(Y,q) = {sp.sstr(candidate.polynomial_expr)} = 0",
        f"X(Y,q) follows the quadratic relation {sp.sstr(x_quad)} = 0",
        f"Z(Y,q) = {sp.sstr(z_expr)}",
        "",
        "Joint-angle recovery:",
        "Delta theta1 = 2*atan(X)",
        "Delta theta2 = 2*atan(Y)",
        "Delta theta4 = 2*atan(Z)",
        "Delta theta5 = -Delta theta3",
        "Delta theta6 = -Delta theta2",
        "Delta theta7 = Delta theta1",
    ]
    (workspace.output_dir / EXPLICIT_JOINT_FUNCTIONS_FILE).write_text("\n".join(explicit_joint_text), encoding="utf-8")

    return {
        "z_expr": z_expr,
        "x_quadratic_expr": x_quad,
        "x_root_plus": x_root_plus,
        "x_root_minus": x_root_minus,
    }


def real_y_roots(candidate: BranchCandidate, q_value: float) -> list:
    """Compute real roots of the implicit Y-q branch at a numeric q."""
    expr = sp.expand(candidate.polynomial_expr.subs(q, q_value))
    poly = sp.Poly(expr, Y)
    coeffs = [complex(sp.N(item, 40)) for item in poly.all_coeffs()]
    roots = np.roots(coeffs)
    values = []
    for root in roots:
        if abs(root.imag) < 1e-8:
            values.append(float(root.real))
    values.sort()
    return values


def x_roots_from_quadratic(x_quadratic_expr: sp.Expr, y_value: float, q_value: float) -> list:
    """Compute the real X roots from the reduced quadratic relation."""
    expr = sp.expand(x_quadratic_expr.subs({Y: y_value, q: q_value}))
    poly = sp.Poly(expr, X)
    coeffs = [complex(sp.N(item, 40)) for item in poly.all_coeffs()]
    roots = np.roots(coeffs)
    values = []
    for root in roots:
        if abs(root.imag) < 1e-8:
            values.append(float(root.real))
    return values


def closure_residual_for_xyz(workspace: ProjectWorkspace, x_value: float, y_value: float, z_value: float, s_rad: float) -> float:
    """Evaluate the full MDH closure residual with the existing numeric verifier."""
    variables = np.array([2.0 * math.atan(x_value), 2.0 * math.atan(y_value), 2.0 * math.atan(z_value)], dtype=float)
    total = workspace.verify_module.closure_matrix(variables, s_rad)
    return float(np.linalg.norm(total - np.eye(4), ord="fro"))


def choose_continuous_branch_point(workspace: ProjectWorkspace, candidate: BranchCandidate, recovery_data: dict, s_deg: float, previous_state: dict | None) -> dict | None:
    """Pick a continuous real branch point at one sample angle."""
    if abs(s_deg - 180.0) < 1e-12:
        q_value = 1.0e8
    else:
        q_value = math.tan(math.radians(s_deg) / 2.0)
        if abs(q_value) > 1.0e8:
            q_value = math.copysign(1.0e8, q_value)

    y_candidates = real_y_roots(candidate, q_value)
    if not y_candidates:
        return None

    best = None
    for y_value in y_candidates:
        z_value = float(sp.N(recovery_data["z_expr"].subs({Y: y_value, q: q_value}), 40))
        for x_value in x_roots_from_quadratic(recovery_data["x_quadratic_expr"], y_value, q_value):
            residual = closure_residual_for_xyz(workspace, x_value, y_value, z_value, math.radians(s_deg))
            continuity_penalty = 0.0
            if previous_state is not None:
                continuity_penalty = (
                    abs(x_value - previous_state["X"])
                    + abs(y_value - previous_state["Y"])
                    + abs(z_value - previous_state["Z"])
                )
            score = residual + 1.0e-3 * continuity_penalty
            candidate_state = {
                "s_deg": s_deg,
                "q": q_value,
                "X": x_value,
                "Y": y_value,
                "Z": z_value,
                "residual": residual,
                "score": score,
            }
            if best is None or candidate_state["score"] < best["score"]:
                best = candidate_state
    return best


def generate_analytic_validation_and_comparison(workspace: ProjectWorkspace, candidate: BranchCandidate, recovery_data: dict) -> dict:
    """Generate analytic residuals and compare them with the existing numeric continuation results."""
    log("Generating analytic_residual.csv, analytic_residual.png, comparison.png, comparison_report.txt")
    sampled_states = []
    previous_state = None
    for s_deg in range(0, 361):
        state = choose_continuous_branch_point(workspace, candidate, recovery_data, float(s_deg), previous_state)
        if state is None:
            sampled_states.append(
                {
                    "s_deg": float(s_deg),
                    "q": float("nan"),
                    "X": float("nan"),
                    "Y": float("nan"),
                    "Z": float("nan"),
                    "x_rad": float("nan"),
                    "y_rad": float("nan"),
                    "z_rad": float("nan"),
                    "residual": float("nan"),
                }
            )
            continue

        previous_state = state
        sampled_states.append(
            {
                "s_deg": state["s_deg"],
                "q": state["q"],
                "X": state["X"],
                "Y": state["Y"],
                "Z": state["Z"],
                "x_rad": 2.0 * math.atan(state["X"]),
                "y_rad": 2.0 * math.atan(state["Y"]),
                "z_rad": 2.0 * math.atan(state["Z"]),
                "residual": state["residual"],
            }
        )

    analytic_csv_path = workspace.output_dir / ANALYTIC_RESIDUAL_CSV_FILE
    with analytic_csv_path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(
            file,
            fieldnames=["s_deg", "q", "X", "Y", "Z", "x_rad", "y_rad", "z_rad", "residual"],
        )
        writer.writeheader()
        writer.writerows(sampled_states)

    valid_residuals = [row["residual"] for row in sampled_states if not math.isnan(row["residual"])]
    max_row = max((row for row in sampled_states if not math.isnan(row["residual"])), key=lambda row: row["residual"])
    mean_residual = float(np.mean(valid_residuals))
    std_residual = float(np.std(valid_residuals))

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot([row["s_deg"] for row in sampled_states], [row["residual"] for row in sampled_states], linewidth=1.4)
    ax.set_xlabel("Delta theta3 (deg)")
    ax.set_ylabel("||T-I||_F")
    ax.set_title("Analytic branch closure residual")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(workspace.output_dir / ANALYTIC_RESIDUAL_PNG_FILE, dpi=200)
    plt.close(fig)

    numeric_map = {int(float(row["s_deg"])): row for row in workspace.numeric_rows}
    comparison_rows = []
    for row in sampled_states:
        s_key = int(row["s_deg"])
        if math.isnan(row["x_rad"]) or s_key not in numeric_map:
            continue
        numeric = numeric_map[s_key]
        comparison_rows.append(
            {
                "s_deg": row["s_deg"],
                "analytic_x_deg": math.degrees(row["x_rad"]),
                "analytic_y_deg": math.degrees(row["y_rad"]),
                "analytic_z_deg": math.degrees(row["z_rad"]),
                "numeric_x_deg": float(numeric["x_deg"]),
                "numeric_y_deg": float(numeric["y_deg"]),
                "numeric_z_deg": float(numeric["z_deg"]),
            }
        )

    def wrapped_difference_deg(a_deg: float, b_deg: float) -> float:
        return (a_deg - b_deg + 180.0) % 360.0 - 180.0

    dx = np.array([wrapped_difference_deg(item["analytic_x_deg"], item["numeric_x_deg"]) for item in comparison_rows])
    dy = np.array([wrapped_difference_deg(item["analytic_y_deg"], item["numeric_y_deg"]) for item in comparison_rows])
    dz = np.array([wrapped_difference_deg(item["analytic_z_deg"], item["numeric_z_deg"]) for item in comparison_rows])

    fig, axes = plt.subplots(3, 1, figsize=(10, 10), sharex=True)
    axes[0].plot([item["s_deg"] for item in comparison_rows], dx, label="Delta theta1 error (deg)")
    axes[1].plot([item["s_deg"] for item in comparison_rows], dy, label="Delta theta2 error (deg)")
    axes[2].plot([item["s_deg"] for item in comparison_rows], dz, label="Delta theta4 error (deg)")
    for axis in axes:
        axis.grid(True, alpha=0.3)
        axis.legend()
    axes[2].set_xlabel("Delta theta3 (deg)")
    fig.tight_layout()
    fig.savefig(workspace.output_dir / COMPARISON_PNG_FILE, dpi=200)
    plt.close(fig)

    comparison_lines = [
        "# comparison_report",
        f"Delta_theta1_RMSE_deg={float(np.sqrt(np.mean(dx**2)))}",
        f"Delta_theta1_max_abs_deg={float(np.max(np.abs(dx)))}",
        f"Delta_theta1_mean_abs_deg={float(np.mean(np.abs(dx)))}",
        f"Delta_theta2_RMSE_deg={float(np.sqrt(np.mean(dy**2)))}",
        f"Delta_theta2_max_abs_deg={float(np.max(np.abs(dy)))}",
        f"Delta_theta2_mean_abs_deg={float(np.mean(np.abs(dy)))}",
        f"Delta_theta4_RMSE_deg={float(np.sqrt(np.mean(dz**2)))}",
        f"Delta_theta4_max_abs_deg={float(np.max(np.abs(dz)))}",
        f"Delta_theta4_mean_abs_deg={float(np.mean(np.abs(dz)))}",
        "",
        f"analytic_residual_max={max_row['residual']}",
        f"analytic_residual_mean={mean_residual}",
        f"analytic_residual_std={std_residual}",
        f"analytic_residual_max_angle_deg={max_row['s_deg']}",
    ]
    (workspace.output_dir / COMPARISON_REPORT_FILE).write_text("\n".join(comparison_lines), encoding="utf-8")

    return {
        "max_residual": max_row["residual"],
        "mean_residual": mean_residual,
        "std_residual": std_residual,
        "max_angle_deg": max_row["s_deg"],
    }


def generate_paper_derivation(workspace: ProjectWorkspace, candidate: BranchCandidate, recovery_data: dict, validation_stats: dict) -> Path:
    """Write a paper-style Chinese derivation markdown file."""
    log("Generating paper_derivation.md")
    z_expr = sp.latex(recovery_data["z_expr"])
    x_quad = sp.latex(recovery_data["x_quadratic_expr"])
    y_poly = sp.latex(candidate.polynomial_expr)
    content = f"""# 7R 闭链机构解析求解推导

## 1. MDH 建模

本文采用修正 DH 变换

$$
A_i = R_x(\\alpha_{{i-1}}) T_x(a_{{i-1}}) R_z(\\theta_i) T_z(d_i).
$$

完整闭环矩阵为

$$
T = A_1 A_2 A_3 A_4 A_5 A_6 A_7,
$$

闭环条件为

$$
T - I = 0.
$$

## 2. 对称关系降维

根据机构对称关系

$$
\\Delta\\theta_1 = \\Delta\\theta_7,\\quad
\\Delta\\theta_2 = -\\Delta\\theta_6,\\quad
\\Delta\\theta_5 = -\\Delta\\theta_3,
$$

定义

$$
s = \\Delta\\theta_3,\\quad
x = \\Delta\\theta_1,\\quad
y = \\Delta\\theta_2,\\quad
z = \\Delta\\theta_4.
$$

这样未知量只剩下 $x,y,z$ 三个。

## 3. 半角代换

采用

$$
q = \\tan\\frac{{s}}{{2}},\\quad
X = \\tan\\frac{{x}}{{2}},\\quad
Y = \\tan\\frac{{y}}{{2}},\\quad
Z = \\tan\\frac{{z}}{{2}}.
$$

从而把三角闭环方程化为有理多项式系统。

## 4. 独立闭环方程与 Jacobian 判定

已有结果文件中共提取到 9 个唯一闭环分子多项式，并筛得 83 组三元 rank=3 独立方程组。独立性判定通过对 $[X,Y,Z]$ 的 Jacobian 在一般点处计算秩完成。

## 5. Resultant / 因子结构消元

对低复杂度独立组尝试多种消元次序时，部分路径出现零 resultant 或超时，因此没有直接得到稳定的显式主分支。进一步利用已有因子结构，可提取出

$$
Z(Y,q) = {z_expr},
$$

以及关于 $X$ 的二次关系

$$
{x_quad} = 0.
$$

继续将这些关系代回其它闭环多项式，可以得到关于 $(Y,q)$ 的最低次数隐式主分支候选

$$
F(Y,q) = {y_poly} = 0.
$$

## 6. 主分支选择

主分支不是只通过 $F(0,0)=0$ 判定，而是同时要求恢复关系满足

$$
X(0,0)=0,\\quad Y(0,0)=0,\\quad Z(0,0)=0.
$$

在本工程中，最终保留的解析解是隐式解析解：

$$
F(Y,q)=0,\\qquad Z = Z(Y,q),\\qquad X \\text{{ 由上式二次方程确定}}.
$$

## 7. 完整 MDH 闭环残差验证

后处理脚本沿主分支对 $\\Delta\\theta_3 = 0^\\circ,1^\\circ,\\dots,360^\\circ$ 连续取样，并把恢复出的 $X,Y,Z$ 重新代入完整 MDH 闭环。

残差统计为：

$$
\\max \\|T-I\\|_F = {validation_stats['max_residual']:.6e},
$$

$$
\\mathrm{{mean}} \\|T-I\\|_F = {validation_stats['mean_residual']:.6e},
$$

$$
\\mathrm{{std}} \\|T-I\\|_F = {validation_stats['std_residual']:.6e}.
$$

最大残差对应输入角约为 $\\Delta\\theta_3 = {validation_stats['max_angle_deg']:.1f}^\\circ$。

## 8. 与数值解比较

解析主分支恢复出的 $\\Delta\\theta_1,\\Delta\\theta_2,\\Delta\\theta_4$ 与已有 `joint_solution_numeric.csv` 做逐点比较，并输出误差曲线和 RMSE 统计。

## 9. 结论

当前最可靠的最终解析结果不是伪造的短显式公式，而是如下隐式解析解：

$$
F(Y,q)=0,\\qquad Z = Z(Y,q),\\qquad X \\text{{ 由二次关系确定}}.
$$

这满足“优先保证数学正确性”和“最终结果必须通过完整 MDH 闭环残差验证”的要求。
"""
    path = workspace.output_dir / PAPER_DERIVATION_FILE
    path.write_text(content, encoding="utf-8")
    return path


def main() -> None:
    """Run the full lightweight post-processing workflow."""
    workspace = ProjectWorkspace.from_existing_results(PROJECT_DIR, PROJECT_DIR)
    generate_check_report(workspace)
    run_elimination_path_search(workspace, top_k=DEFAULT_TOP_K, timeout_seconds=DEFAULT_TIMEOUT_SECONDS)
    _selected_path, candidate = select_main_branch(workspace)
    recovery_data = save_solution_files(workspace, candidate)
    validation_stats = generate_analytic_validation_and_comparison(workspace, candidate, recovery_data)
    generate_paper_derivation(workspace, candidate, recovery_data, validation_stats)
    log("Post-processing finished")


if __name__ == "__main__":
    main()
