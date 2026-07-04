# symbolic_mdh_branch_solution

## MDH formula

This directory uses the modified DH convention:

```text
A_i = Rx(alpha_{i-1}) * Tx(a_{i-1}) * Rz(theta_i) * Tz(d_i)
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
- Any triplet that exceeds 60 seconds is skipped and recorded.
- Numeric verification starts with the full matrix residual and falls back to translation plus vee((R-R^T)/2) when needed.
