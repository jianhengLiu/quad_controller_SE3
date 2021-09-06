# Geometric Controls of a Quadrotor UAV with Decoupled Yaw Control

原文坐标系为Forward-Right-Down坐标系

![image-20210503200519094](Geometric%20Controls%20of%20a%20Quadrotor%20UAV%20with%20Decoupled%20Yaw%20Control.assets/image-20210503200519094.png)

以下为Forward-Left-Up坐标系的推导

**运动方程**
$$
\begin{equation}
m \dot{v}=- m g e_{3} + f R e_{3}+\Delta_{x} 
\end{equation}\label{eq3}
$$

$$
\begin{equation}
J \dot{\Omega}+\Omega \times J \Omega=M+\Delta_{R}
\end{equation}
$$

## 跟踪控制问题

**期望轨迹：**
$$
x_d(t)\in\mathbb{R}^3
$$
**期望方向（机体的第一轴/x轴）**
$$
b_{1_d}(t)\in\mathbf{S}^2=\{q\in\mathbb{R}^3| \|q\|=1\}
$$


**假设1：机体质量分布对称**

即惯性矩阵$J=diag[J_1,J_1,J_3]$ for $J_1,J_3>0$

该假设一般都成立，用于分离yaw角的动力学

**假设2：位置动态不确定性有界**
$$
\|\Delta_x\|\le\delta_x,
\\
\delta_x>0
$$
用于在稳定性分析时限制姿态控制对平移动力学的影响

**假设3：**
$$
\|-mge_3-m\ddot x_d\|\le B_1,
\\
B_1>0
$$
表明期望轨迹的加速度是有界的



基于以上假设，期望设计每一个电机的拉力$(f_1,f_2,f_3,f_4)$使得$x(t)=x_d(t)$指数稳定

# 位置控制

输出的拉力为$fRe_3$，其幅值为$f$，拉力方向为$Re_3$，因此控制位置其姿态也要一起控制。

在这放松约束，假设控制拉力可以被随意调整，即假设$fRe_3$可以被虚拟控制输入$A\in\mathbb{R}^3$代替

**误差变量：**
$$
\begin{array}{l}
e_{x}=x-x_{d} \\
e_{v}=v-\dot{x}_{d}
\end{array}
$$
**积分变量（可选择不用）**

这里的积分变量为论文选择的，可以为其他形式
$$
e_{i}(t)=\int_{0}^{t} e_{v}(\tau)+c_{1} e_{x}(\tau) d \tau
$$
令，世界坐标系的虚拟拉力（即电机拉力在世界坐标系的作用表现）
$$
A=-k_{x} e_{x}-k_{v} e_{v}-k_{i} \operatorname{sat}_{\sigma}\left(e_{i}\right)+m g e_{3}+m \ddot{x}_{d}\label{eq14}
$$
其中，饱和函数如下
$$
\operatorname{sat}_{\sigma}(y)=\left\{\begin{array}{ll}
\sigma & \text { if } y>\sigma \\
y & \text { if }-\sigma \leq y \leq \sigma \\
-\sigma & \text { if } y<-\sigma
\end{array}\right.
$$

## **李雅普诺夫证明**

令$f R e_{3} = A$，将$(\ref{eq14})$代入$(\ref{eq3})$
$$
m \dot{e}_{v}=-k_{x} e_{x}-k_{v} e_{v}-k_{i} \operatorname{sat}_{\sigma}\left(e_{i}\right)+\Delta_{x}
$$
**李雅普诺夫方程**
$$
\begin{aligned}
\mathcal{V}_{x}=& \frac{1}{2} m\left\|e_{v}\right\|^{2}+c_{1} m e_{v} \cdot e_{x}+\frac{1}{2} k_{x}\left\|e_{x}\right\|^{2} \\
&+\int_{\frac{\Delta_{x}}{k_{i}}}^{e_{i}}\left(k_{i} \operatorname{sat}_{\sigma}(\mu)-\Delta_{x}\right) \cdot d \mu
\end{aligned}
$$
在平衡点$(e_x,e_v,e_i)=(0,0,\frac{\Delta_x}{k_i})$处是正定的
$$
\dot{\mathcal{V}}_{x}=-\left(k_{v}-c_{1} m\right)\left\|e_{v}\right\|^{2}-c_{1} k_{v} e_{x} \cdot e_{v}-c_{1} k_{x}\left\|e_{x}\right\|^{2}
$$
对于$c_1$满足以下
$$
\begin{array}{c}
k_{i} \sigma>\delta_{x} \\
c_{1}<\min \left\{\sqrt{\frac{k_{x}}{m}}, \frac{4 k_{x} k_{v}}{k_{v}^{2}+4 m k_{x}}\right\}
\end{array}
$$
李雅普诺夫方程的导数是负半定的，所以依据LaSalle-Yoshizawa论据，$(e_x,e_v)\to(0,0)$ as $t\to\infin$



**升力**

依据动力学，选择总拉力为
$$
f=A\cdot b_3
$$




## 期望旋转矩阵及角速度





# Roll/Pitch Control System

由前可知位置控制是渐进稳定的，但无人机的姿态不可能即时改变，因此需要设计控制系统，使得$f R e_{3}\to A$ as $t\to\infin$

由$f R e_{3}$ 可知，位置控制并不需要全部的姿态控制，而仅需要机体坐标系的第三轴即z轴需要被控制，$b_3=Re_3\in S^2$

**期望方向$b_{3_d}$**
$$
b_{3_d}=\frac{A}{\|A\|}
$$
为了保证$b_3\to b_{3_d},t\to\infin$，即对应roll/pitch控制，因此设计一个roll/pitch控制器控制头两个扭矩$(M_1,M_2)$去控制$b_3$，剩下的$M_3$用于控制yaw的控制器会在单独一节展示

