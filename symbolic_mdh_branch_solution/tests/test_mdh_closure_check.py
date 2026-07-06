import math
import sys
from pathlib import Path

import numpy as np


PROJECT_DIR = Path(__file__).resolve().parents[1]
if str(PROJECT_DIR) not in sys.path:
    sys.path.insert(0, str(PROJECT_DIR))


def test_initial_configuration_is_identity():
    import mdh_closure_check as closure_check

    total = closure_check.build_total_transform(
        theta1=0.0,
        theta2=0.0,
        theta3=math.pi,
        theta4=0.0,
        theta5=0.0,
        theta6=math.pi,
        theta7=0.0,
        length_scale=1.0,
    )

    assert np.allclose(total, np.eye(4), atol=1e-12)
