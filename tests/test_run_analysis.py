from pathlib import Path

import matplotlib

matplotlib.use("Agg")

from planar_fourbar_dh.run_analysis import run_analysis


def test_analysis_writes_expected_outputs(tmp_path: Path):
    """分析入口必须生成数值表和三类验证图。"""
    outputs = run_analysis(output_dir=tmp_path, steps=181)
    expected = {
        "joint_angles.csv",
        "joint_angle_mapping.png",
        "joint_angle_mapping.svg",
        "joint_angle_mapping.pdf",
        "closure_error.png",
        "fourbar_configurations.png",
    }
    assert expected.issubset({path.name for path in outputs})
    assert all(path.stat().st_size > 0 for path in outputs)
