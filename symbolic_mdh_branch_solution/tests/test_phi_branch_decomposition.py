import csv
import sys
from pathlib import Path


PROJECT_DIR = Path(__file__).resolve().parents[1]
if str(PROJECT_DIR) not in sys.path:
    sys.path.insert(0, str(PROJECT_DIR))


def test_phi_branch_decomposition_writes_all_branch_outputs(tmp_path):
    import validate_phi_branches as validator

    output_dir = tmp_path / "phi_branch_decomposition"
    validator.run_phi_branch_decomposition(output_dir=output_dir, sample_count=41)

    expected_files = [
        "branch_decomposition_report.txt",
        "branch_decomposition.csv",
        "branch1_angles.png",
        "branch2_angles.png",
        "branch3_angles.png",
        "branch4_angles.png",
        "branch_residuals.png",
    ]
    for filename in expected_files:
        assert (output_dir / filename).exists(), filename

    with (output_dir / "branch_decomposition.csv").open("r", encoding="utf-8", newline="") as file:
        rows = list(csv.DictReader(file))

    assert len(rows) == 4
    key = list(rows[0].keys())[0]
    assert {row[key] for row in rows} == {"Branch 1", "Branch 2", "Branch 3", "Branch 4"}
