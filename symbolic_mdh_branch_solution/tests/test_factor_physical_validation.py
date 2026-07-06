import csv
import sys
from pathlib import Path


PROJECT_DIR = Path(__file__).resolve().parents[1]
if str(PROJECT_DIR) not in sys.path:
    sys.path.insert(0, str(PROJECT_DIR))


def test_factor_validation_writes_summary_for_all_known_factors(tmp_path):
    import validate_factor_physical_meaning as validator

    output_dir = tmp_path / "factor_physical_validation"
    validator.run_factor_validation(output_dir=output_dir, sample_count=21)

    expected_files = [
        "factor_validation_summary.csv",
        "factor_validation_report.md",
        "factor_F1_branch_curves.csv",
        "factor_F1_branch_curves.png",
    ]
    for filename in expected_files:
        assert (output_dir / filename).exists(), filename

    with (output_dir / "factor_validation_summary.csv").open("r", encoding="utf-8", newline="") as file:
        rows = list(csv.DictReader(file))

    assert len(rows) == 6
    factor_key = list(rows[0].keys())[0]
    assert {row[factor_key] for row in rows} == {
        "q=0",
        "Y=1",
        "Y=-1",
        "q=1",
        "Y^2*q-4*Y^2-4*q+1=0",
        "Phi(Y,q)=0",
    }
