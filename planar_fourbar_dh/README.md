# 2–4–6–1 平面四杆显式解

本程序沿用原七杆机构的改进 DH 参数。默认初始关节变量为

```text
q0 = [0, 0, 0, 0, 0, 0, 0]
```

第 3、6 关节的 `pi` 已作为 DH 固定偏置计入实际变换。平面模式中关节
3、5、7 锁定在各自初始值，关节 2 为输入，解析计算关节 4、5、7。

## 运行

```powershell
python -m planar_fourbar_dh.run_analysis
```

指定输入范围和输出目录：

```powershell
python -m planar_fourbar_dh.run_analysis `
  --output-dir planar_fourbar_output `
  --steps 721 `
  --q2-start-deg 0 `
  --q2-end-deg 360 `
  --length-scale 1
```

依赖：

```powershell
python -m pip install numpy matplotlib pytest
```

## 显式闭环

给定输入角后，先计算关节 4 的轴心位置。关节 6 的轴心同时位于：

- 以关节 4 为圆心、半径 `L46` 的圆；
- 以关节 1 为圆心、半径 `L61` 的圆。

设两圆心距离为 `D`，则交点为

```text
P6 = P4 + x*e + s*h*e_perp
x  = (L46^2 - L61^2 + D^2) / (2D)
h  = sqrt(L46^2 - x^2)
```

其中 `s = ±1` 对应两条装配支路。程序自动选择经过 DH 初始姿态的支路。

程序再利用四根关节轴相对公共法向的方向符号，将平面杆件绝对方向换算为
原七杆 DH 关节角。二者不能直接等同。

## 输出

- `joint_angles.csv`：输入角、`q4/q6/q1` 和闭环误差；
- `joint_angle_mapping.png/svg/pdf`：关节角显式映射；
- `closure_error.png`：完整七杆 DH 闭环误差；
- `fourbar_configurations.png`：四个典型输入位置下的机构构型。
