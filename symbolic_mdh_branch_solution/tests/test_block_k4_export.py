import csv
import sys
from pathlib import Path


PROJECT_DIR = Path(__file__).resolve().parents[1]
if str(PROJECT_DIR) not in sys.path:
    sys.path.insert(0, str(PROJECT_DIR))


def test_block_k4_export_writes_all_required_files(tmp_path):
    import derive_block_k4_equations as block_k4

    output_dir = tmp_path / "block_k4_outputs"
    block_k4.export_block_k4_bundle(output_dir)

    expected_files = [
        "block_k4_left_matrix.txt",
        "block_k4_right_matrix.txt",
        "block_k4_equations_trig.txt",
        "block_k4_equations_half_angle.txt",
        "block_k4_equation_summary.csv",
        "block_k4_equations_latex.tex",
        "README.md",
    ]
    for filename in expected_files:
        assert (output_dir / filename).exists(), filename

    trig_text = (output_dir / "block_k4_equations_trig.txt").read_text(encoding="utf-8")
    assert trig_text.count("编号：E") == 16

    with (output_dir / "block_k4_equation_summary.csv").open("r", encoding="utf-8", newline="") as file:
        rows = list(csv.DictReader(file))
    assert len(rows) == 16
    assert {row["编号"] for row in rows} == {f"E{i}{j}" for i in range(1, 5) for j in range(1, 5)}
