# 2.3 退化平面分支的关节角关系推导底稿

本节不再从一般空间闭环的数值约束出发，而只针对机构退化为平面分支时的关节协调关系进行推导。机构总体仍按改进型DH参数建模，其单关节齐次变换写为

$$
{}^{i-1}\mathbf T_i=
\begin{bmatrix}
\cos\theta_i & -\sin\theta_i & 0 & a_i \\
\sin\theta_i\cos\alpha_i & \cos\theta_i\cos\alpha_i & -\sin\alpha_i & -d_i\sin\alpha_i \\
\sin\theta_i\sin\alpha_i & \cos\theta_i\sin\alpha_i & \cos\alpha_i & d_i\cos\alpha_i \\
0 & 0 & 0 & 1
\end{bmatrix},
$$

其中各关节的结构参数取为

$$
\begin{aligned}
(\alpha_1,a_1,d_1,\theta_{1,0})&=(0,l,0,0),\\
(\alpha_2,a_2,d_2,\theta_{2,0})&=\left(\frac{\pi}{2},2l,2.5l,0\right),\\
(\alpha_3,a_3,d_3,\theta_{3,0})&=\left(\frac{\pi}{2},0,0,\pi\right),\\
(\alpha_4,a_4,d_4,\theta_{4,0})&=\left(\frac{\pi}{2},2.5l,0,0\right),\\
(\alpha_5,a_5,d_5,\theta_{5,0})&=\left(\frac{\pi}{2},2.5l,0,0\right),\\
(\alpha_6,a_6,d_6,\theta_{6,0})&=\left(\frac{\pi}{2},0,2.5l,\pi\right),\\
(\alpha_7,a_7,d_7,\theta_{7,0})&=\left(\frac{\pi}{2},2l,0,0\right),
\end{aligned}
$$

并记实际关节角为 $\theta_i=q_i+\theta_{i,0}$。在退化平面分支上，机构中仅有一组互相平行的转轴参与运动，空间 7R 闭环退化为一个单自由度平面四杆回路。按照几何约束可取参与该平面运动的四个转角为 $\theta_1,\theta_2,\theta_4,\theta_6$，其余关节在该分支上不再提供独立自由度，因此整个分支的构型可由一个独立参数唯一确定。为使后续推导尽量紧凑，将原空间闭环中的尺寸统一写成

$$
l_0=l,\qquad a=2l,\qquad b=\frac{5}{2}l,
$$

则退化后的等效四杆边长满足

$$
l_1=l_3=\sqrt{a^2+b^2},\qquad l_2=2a+l=5l.
$$

又由于斜边方向与基准方向之间存在固定几何偏角，定义

$$
\beta=\operatorname{atan2}(b,a).
$$

这样处理以后，原空间机构的退化平面分支便可转化为一个以固定杆 $l_0$、输入杆 $l_1$、连杆 $l_2$ 和输出杆 $l_3$ 组成的四杆机构。取 $\theta_1$ 作为分支参数，并将其相对于斜边方向的几何角记为

$$
\varphi_1=\theta_1-\beta.
$$

于是输入杆末端点的平面坐标可写为

$$
\mathbf p_3=
\begin{bmatrix}
l_1\cos\varphi_1\\
l_1\sin\varphi_1
\end{bmatrix}.
$$

该点到另一固定铰点的距离为

$$
d=\sqrt{(l_1\cos\varphi_1-l_0)^2+(l_1\sin\varphi_1)^2},
$$

相应的极角为

$$
\alpha=\operatorname{atan2}\!\left(l_1\sin\varphi_1,\;l_1\cos\varphi_1-l_0\right).
$$

在由 $l_2,l_3,d$ 构成的闭环三角形中，由余弦定理有

$$
\gamma=\arccos\!\left(\frac{l_3^2+d^2-l_2^2}{2l_3d}\right).
$$

因此输出杆相对于固定杆的几何方向角为

$$
\varphi_3=\alpha-\gamma.
$$

进一步地，由输入杆端点与输出杆端点之间的矢量关系，可得中间连杆的方向角

$$
\varphi_2=
\operatorname{atan2}\!\left(
l_3\sin\varphi_3-l_1\sin\varphi_1,\;
l_0+l_3\cos\varphi_3-l_1\cos\varphi_1
\right).
$$

至此，平面四杆的三个几何角 $\varphi_1,\varphi_2,\varphi_3$ 均已被表示成 $\theta_1$ 的函数。下面只需将这三个几何角重新映射回原机构的关节变量即可得到退化分支上的关节协调关系。根据初始装配位形与各关节零位偏置的定义，输入侧关节角、连杆转角和输出侧关节角之间满足

$$
\theta_2=\theta_1-\varphi_2+\pi,
$$

$$
\theta_4=\varphi_3-\varphi_2-\beta,
$$

$$
\theta_6=-\theta_1+\theta_2-\theta_4.
$$

将上式中 $\varphi_2$ 与 $\varphi_3$ 的表达式代入后，可得 $\theta_2,\theta_4,\theta_6$ 全部由独立参数 $\theta_1$ 唯一确定，即

$$
\theta_2=f_2(\theta_1),\qquad
\theta_4=f_4(\theta_1),\qquad
\theta_6=f_6(\theta_1).
$$

于是，退化平面分支上的全部关节协调关系被压缩为一条一维参数曲线

$$
\mathbf q_{\mathrm{planar}}(\theta_1)=
\begin{bmatrix}
\theta_1 & \theta_2(\theta_1) & \theta_4(\theta_1) & \theta_6(\theta_1)
\end{bmatrix}^{\mathrm T}.
$$

这说明该分支并不是瞬时的偶然共面构型，而是一条具有确定连续参数化表达的真实单自由度运动分支。若将驱动改写为另一物理关节，例如采用 $\theta_2$ 或 $\theta_6$ 作为输入，则闭式反求 $\theta_1(\theta_2)$ 或 $\theta_1(\theta_6)$ 往往不再紧凑，此时更适合沿上述显式分支进行连续延拓数值求解，再将求得的关节角序列展开为单值曲线。也就是说，理论推导部分给出了该退化分支存在且为单自由度的几何依据，而数值连续追踪只是将这条显式参数曲线重新投影到不同驱动变量下的坐标表示。后续若需验证该分支在整周运动中的真实性与奇异位置，只需在该参数曲线上逐点构造被动雅可比矩阵 $\mathbf J_p$ 并计算其最小奇异值；当 $\sigma_{\min}(\mathbf J_p)=0$ 时，对应构型即为该退化分支与其他运动分支发生交汇的真实奇异位形。

可直接用于正文的压缩结论句可写为：在退化平面分支上，空间 7R 闭环可约化为单自由度平面四杆回路；以 $\theta_1$ 为独立参数时，可先由几何关系求得 $\varphi_1,\varphi_2,\varphi_3$，再映射得到 $\theta_2=f_2(\theta_1)$、$\theta_4=f_4(\theta_1)$ 和 $\theta_6=f_6(\theta_1)$，从而将该分支写成具有显式关节协调关系的一维参数曲线。
