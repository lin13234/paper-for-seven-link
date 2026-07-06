# 7R 闭链机构 MDH 闭环方程整理

## 1. 建模定义

采用的 modified DH 形式为

$$
A_i = R_x(\alpha_{i-1})\,T_x(a_{i-1})\,R_z(\theta_i)\,T_z(d_i)
$$

完整闭环矩阵定义为

$$
T = A_1A_2A_3A_4A_5A_6A_7
$$

闭环方程定义为

$$
F = T - I
$$

这里列出的方程，不是直接把 $F$ 的 16 个元素原样抄出，而是按照当前项目里已经做过的处理方式：

1. 先对 $F$ 的 16 个元素做半角代换；
2. 提取每个非零元素的分子多项式；
3. 去重；
4. 记录得到的 9 个唯一闭环多项式。

## 2. 半角变量

$$
q=\tan\frac{s}{2},\quad
X=\tan\frac{x}{2},\quad
Y=\tan\frac{y}{2},\quad
Z=\tan\frac{z}{2}
$$

其中

$$
\theta_1=x,\quad
\theta_2=y,\quad
\theta_3=\pi+s,\quad
\theta_4=z,\quad
\theta_5=-s,\quad
\theta_6=\pi-y,\quad
\theta_7=x
$$

## 3. 常见因子记号

为了避免后面重复太长，先定义几个反复出现的因子：

$$
\Phi_1 =
X^2Y^2q + X^2q + XY^2q^2 - XY^2 - 2XYZq^2 - 2XYZ - Xq^2 + X - Y^2q - q
$$

$$
\Phi_2 =
Y^2Zq^2 + Y^2Z + 2Yq^2 - 2Y - Zq^2 - Z
$$

$$
\Phi_3 =
X^2Y^2q^2 - X^2Y^2 - 2X^2YZq^2 - 2X^2YZ - X^2q^2 + X^2
- 4XY^2q - 4Xq - Y^2q^2 + Y^2 + 2YZq^2 + 2YZ + q^2 - 1
$$

$$
\Phi_4 =
Y^2q^2 - 10Y^2q + 9Y^2 + 8YZq^2 + 8YZ + 9q^2 - 10q + 1
$$

## 4. 去重后的 9 个闭环多项式

### Eq 1

$$
\begin{aligned}
E_1={}&X^4Y^4Z^2q^4 + 2X^4Y^4Z^2q^2 + X^4Y^4Z^2 + 4X^4Y^4q^2 + 4X^4Y^3Zq^4 - 4X^4Y^3Z \\
&- 2X^4Y^2Z^2q^4 - 4X^4Y^2Z^2q^2 - 2X^4Y^2Z^2 + 4X^4Y^2q^4 + 4X^4Y^2 - 4X^4YZq^4 + 4X^4YZ \\
&+ X^4Z^2q^4 + 2X^4Z^2q^2 + X^4Z^2 + 4X^4q^2 + 8X^3Y^4q^3 - 8X^3Y^4q - 16X^3Y^3Zq^3 - 16X^3Y^3Zq \\
&- 16X^3YZq^3 - 16X^3YZq - 8X^3q^3 + 8X^3q + 2X^2Y^4Z^2q^4 + 4X^2Y^4Z^2q^2 + 2X^2Y^4Z^2 \\
&+ 4X^2Y^4q^4 - 16X^2Y^4q^2 + 4X^2Y^4 - 8X^2Y^3Zq^4 + 8X^2Y^3Z + 12X^2Y^2Z^2q^4 + 24X^2Y^2Z^2q^2 \\
&+ 12X^2Y^2Z^2 - 16X^2Y^2q^2 + 8X^2YZq^4 - 8X^2YZ + 2X^2Z^2q^4 + 4X^2Z^2q^2 + 2X^2Z^2 \\
&+ 4X^2q^4 - 16X^2q^2 + 4X^2 - 8XY^4q^3 + 8XY^4q + 16XY^3Zq^3 + 16XY^3Zq + 16XYZq^3 + 16XYZq \\
&+ 8Xq^3 - 8Xq + Y^4Z^2q^4 + 2Y^4Z^2q^2 + Y^4Z^2 + 4Y^4q^2 + 4Y^3Zq^4 - 4Y^3Z \\
&- 2Y^2Z^2q^4 - 4Y^2Z^2q^2 - 2Y^2Z^2 + 4Y^2q^4 + 4Y^2 - 4YZq^4 + 4YZ + Z^2q^4 + 2Z^2q^2 + Z^2 + 4q^2
\end{aligned}
$$

### Eq 2

$$
E_2=\Phi_1\Phi_3
$$

即

$$
\begin{aligned}
E_2={}&(X^2Y^2q + X^2q + XY^2q^2 - XY^2 - 2XYZq^2 - 2XYZ - Xq^2 + X - Y^2q - q) \\
&\cdot (X^2Y^2q^2 - X^2Y^2 - 2X^2YZq^2 - 2X^2YZ - X^2q^2 + X^2 - 4XY^2q - 4Xq \\
&\qquad - Y^2q^2 + Y^2 + 2YZq^2 + 2YZ + q^2 - 1)
\end{aligned}
$$

### Eq 3

$$
E_3=\Phi_2\Phi_3
$$

即

$$
\begin{aligned}
E_3={}&(Y^2Zq^2 + Y^2Z + 2Yq^2 - 2Y - Zq^2 - Z) \\
&\cdot (X^2Y^2q^2 - X^2Y^2 - 2X^2YZq^2 - 2X^2YZ - X^2q^2 + X^2 - 4XY^2q - 4Xq \\
&\qquad - Y^2q^2 + Y^2 + 2YZq^2 + 2YZ + q^2 - 1)
\end{aligned}
$$

### Eq 4

$$
\begin{aligned}
E_4={}&X^2Y^4Z^2q^4 + 2X^2Y^4Z^2q^2 + X^2Y^4Z^2 + 2X^2Y^4q^4 - 10X^2Y^4q^3 + 10X^2Y^4q^2 + 10X^2Y^4q - 8X^2Y^4 \\
&+ 6X^2Y^3Zq^4 + 20X^2Y^3Zq^3 - 20X^2Y^3Zq^2 + 20X^2Y^3Zq - 26X^2Y^3Z - 14X^2Y^2Z^2q^4 - 28X^2Y^2Z^2q^2 \\
&- 14X^2Y^2Z^2 + 10X^2Y^2q^4 - 12X^2Y^2q^2 + 10X^2Y^2 - 26X^2YZq^4 + 20X^2YZq^3 - 20X^2YZq^2 \\
&+ 20X^2YZq + 6X^2YZ + X^2Z^2q^4 + 2X^2Z^2q^2 + X^2Z^2 - 8X^2q^4 + 10X^2q^3 + 10X^2q^2 - 10X^2q + 2X^2 \\
&- 4XY^4q^3 + 40XY^4q^2 - 36XY^4q - 32XY^3Zq^3 - 32XY^3Zq - 40XY^2q^3 + 80XY^2q^2 - 40XY^2q \\
&- 32XYZq^3 - 32XYZq - 36Xq^3 + 40Xq^2 - 4Xq + Y^4Z^2q^4 + 2Y^4Z^2q^2 + Y^4Z^2 \\
&+ 10Y^4q^3 - 6Y^4q^2 - 10Y^4q + 10Y^4 - 6Y^3Zq^4 - 20Y^3Zq^3 + 20Y^3Zq^2 - 20Y^3Zq + 26Y^3Z \\
&+ 18Y^2Z^2q^4 + 36Y^2Z^2q^2 + 18Y^2Z^2 - 6Y^2q^4 + 20Y^2q^2 - 6Y^2 + 26YZq^4 - 20YZq^3 \\
&+ 20YZq^2 - 20YZq - 6YZ + Z^2q^4 + 2Z^2q^2 + Z^2 + 10q^4 - 10q^3 - 6q^2 + 10q
\end{aligned}
$$

### Eq 5

$$
E_5=\Phi_1^2
$$

即

$$
E_5=
\left(
X^2Y^2q + X^2q + XY^2q^2 - XY^2 - 2XYZq^2 - 2XYZ - Xq^2 + X - Y^2q - q
\right)^2
$$

### Eq 6

$$
E_6=\Phi_2\Phi_1
$$

即

$$
\begin{aligned}
E_6={}&(Y^2Zq^2 + Y^2Z + 2Yq^2 - 2Y - Zq^2 - Z) \\
&\cdot (X^2Y^2q + X^2q + XY^2q^2 - XY^2 - 2XYZq^2 - 2XYZ - Xq^2 + X - Y^2q - q)
\end{aligned}
$$

### Eq 7

$$
E_7=\Phi_4\Phi_1
$$

即

$$
\begin{aligned}
E_7={}&(Y^2q^2 - 10Y^2q + 9Y^2 + 8YZq^2 + 8YZ + 9q^2 - 10q + 1) \\
&\cdot (X^2Y^2q + X^2q + XY^2q^2 - XY^2 - 2XYZq^2 - 2XYZ - Xq^2 + X - Y^2q - q)
\end{aligned}
$$

### Eq 8

$$
E_8=\Phi_2^2
$$

即

$$
E_8=
\left(
Y^2Zq^2 + Y^2Z + 2Yq^2 - 2Y - Zq^2 - Z
\right)^2
$$

### Eq 9

$$
E_9=\Phi_2\Phi_4
$$

即

$$
\begin{aligned}
E_9={}&(Y^2Zq^2 + Y^2Z + 2Yq^2 - 2Y - Zq^2 - Z) \\
&\cdot (Y^2q^2 - 10Y^2q + 9Y^2 + 8YZq^2 + 8YZ + 9q^2 - 10q + 1)
\end{aligned}
$$

## 5. 结构观察

从当前这 9 个去重方程可以直接看出：

1. 真正反复出现的核心因子主要是 $\Phi_1,\Phi_2,\Phi_3,\Phi_4$。
2. `Eq 5 = Eq 2` 的一部分平方结构，`Eq 8` 也是平方结构。
3. `Eq 6, Eq 7, Eq 9` 都是低阶公共因子的乘积。
4. 如果后续只做最小闭环核查，优先盯住这些核心因子比盯住超长多项式更合适。

## 6. 对应原始文件

这份 Markdown 是依据当前项目中的原始方程文件整理得到：

- [equations_unique.txt](/E:/hub/symbolic_mdh_branch_solution/equations_unique.txt)

