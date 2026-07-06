from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path


PROJECT_DIR = Path(__file__).resolve().parents[1] / "symbolic_mdh_branch_solution"
MODULE_PATH = PROJECT_DIR / "postprocess_branch_analysis.py"


def load_module():
    spec = spec_from_file_location("postprocess_branch_analysis", MODULE_PATH)
    module = module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_check_report_can_be_generated(tmp_path):
    """后处理脚本必须能基于已有结果生成检查报告。"""
    module = load_module()
    workspace = module.ProjectWorkspace.from_existing_results(PROJECT_DIR, tmp_path)

    report_path = module.generate_check_report(workspace)

    assert report_path.exists()
    text = report_path.read_text(encoding="utf-8")
    assert "unique_equation_count" in text
    assert "independent_set_count" in text
    assert "main_branch_candidate_count" in text


def test_elimination_path_search_records_statuses(tmp_path):
    """多消元次序搜索必须把 success/zero_resultant/timeout/exception 至少记录为状态字段。"""
    module = load_module()
    workspace = module.ProjectWorkspace.from_existing_results(PROJECT_DIR, tmp_path)

    report_path, records = module.run_elimination_path_search(
        workspace=workspace,
        top_k=3,
        timeout_seconds=1,
        elimination_orders=(("Z", "X"), ("X", "Z")),
    )

    assert report_path.exists()
    assert records
    assert all("status" in record for record in records)
    assert all(record["status"] in {"success", "zero_resultant", "timeout", "exception"} for record in records)


def test_main_branch_screening_checks_xyz_origin_consistency(tmp_path):
    """主分支筛选不能只检查 Y=0,q=0，还必须检查恢复出的 X,Z 在初始位形也为 0。"""
    module = load_module()
    workspace = module.ProjectWorkspace.from_existing_results(PROJECT_DIR, tmp_path)

    bad_candidate = module.BranchCandidate(
        source="unit-test",
        variable="Y",
        polynomial_text="Y",
        polynomial_expr=module.Y,
        metadata={"recovery_x_expr": module.q + 1, "recovery_z_expr": module.Z},
    )
    good_candidate = module.BranchCandidate(
        source="unit-test",
        variable="Y",
        polynomial_text="Y",
        polynomial_expr=module.Y,
        metadata={"recovery_x_expr": module.X, "recovery_z_expr": module.Z},
    )

    bad_result = module.evaluate_branch_candidate(workspace, bad_candidate)
    good_result = module.evaluate_branch_candidate(workspace, good_candidate)

    assert not bad_result["passes_initial_pose"]
    assert good_result["passes_initial_pose"]
