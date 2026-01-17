#import "@preview/physica:0.9.7": *
#set math.equation(numbering: "(1)")
#let fa = $cal(A)$
#let fb = $cal(B)$
#let bl(v1, v2) = $attach(v1, bl: v2)$
= MIT Cheetah 3: Design and Control of a Robust, Dynamic Quadruped Robot

腿部设计
#image("/content/Blog/MIT-Cheetah3/assets/coordinate_system.png", width: 60%)
每个腿有三个自由度,分别由以下三个关节上的驱动器控制,实现对地面接触力的完全三维控制
- abduction/adduction (ab/ad) 外展和内收关节,带动整个腿部绕本体X轴方向转动
- hip 臀部关节,带动整个腿部绕本体Y轴方向转动
- knee 膝关节,带动小腿绕本体Y轴方向转动

控制架构

用户输入期望的水平速度和转向速度, 规划一条质心运动参考轨迹, 根据当前腿的状态使用不同的控制器和步态规划来生成期望的力或者位置控制

步态规划

步态由基于事件的FSM定义, 使用一个phase variable来定义腿处于接触还是摆动阶段(phase), 本文中定义了 trotting, bounding和pacing三种步态, 通过控制每个腿处在不同的phase来模拟动物的步态, 在实际中, 规划的phase和真实(估计)的phase之间可能存在差异,因此用两个变量来区分,根据两个变量的关系可以判断是否发生了提前接触,延迟接触等现象,从而调整控制策略
$
  s_(phi.alt) in {0 = "swing",1="contact"} quad hat(s)_(phi.alt) in {0 = "swing",1="contact"}
$

控制模型

由于四肢惯性相对机体来说非常小,因此在计算地面接触力时不考虑四肢的影响,机体质心加速度和四足接触力(地面施加给足部的力)之间存在线性关系
$
  mat(vb(I)_3, dots.c, vb(I)_3; [vb(p)_1-vb(p)_c]_times, dots.c, [vb(p)_4-vb(p)_c]_times) vec(vb(F)_1, dots.v, vb(F)_4) = vec(m(dot.double(vb(p))_c)+vb(g), vb(I)_G dot(vb(omega))_b) =>\
  vb(A) vb(F) = vb(b)
$

接触力控制 - Balance Controller

对于给定的期望位置/角度以及速度/角速度, 使用PD控制律确定期望的加速度/角加速度
$
  vec(dot.double(vb(p))_(c,d), dot(vb(omega))_(b,d)) = vec(K_(p,p) (vb(p)_(c,d)-vb(p)_c)+K_(d,p) (dot(vb(p))_(c,d)-dot(vb(p))), K_(p,omega) log(vb(R)_d vb(R)^T) + K_(d,omega) (vb(omega)_(b,d) - vb(omega)))
$

基于期望的加速度/角加速度有期望的质心动力学
$
  vb(b)_d = vec(m(dot.double(vb(p))_(c,d))+vb(g), vb(I)_G dot(vb(omega))_(b,d))
$

Balance Controller的目的是求得四足接触力的最优分布以满足期望的质心动力学, 由于动力学方程为线性方程, 该问题可表示为一个二次规划问题 quadratic program (QP)
$
  vb(F)^star = min_vb(F) (vb(A) vb(F) - vb(b)_d)^T vb(S) (vb(A) vb(F) - vb(b)_d) + alpha norm(vb(F))^2 + beta norm(vb(F) - vb(F)^star_"prev")^2 \
  s.t. quad vb(C) vb(F) <= vb(d)
$
代价函数中第一项表示尽可能使质心动力学接近期望值, 同时使用对角矩阵 $vb(S)$ 来表示质心动力学中不同项的相对权重, 第二项表示尽可能使接触力小, 第三项表示尽可能使接触力接近上一步时的接触力分布, 约束条件表明接触力应在摩擦锥内, 即接触点不打滑

除了 Balance Controller 外, 接触力控制还可以采用 Model Predictive Control (MPC), 具体内容参见 _Dynamic Locomotion in the MIT Cheetah 3 Through Convex Model-Predictive Control_ (后续放解读连接)


摆动腿控制

摆动腿落点位置 (2D) 为
$
  vb(p)_"step,i" = vb(p)_"h,i" + T_(c phi.alt)/2 dot(vb(p))_(c,d) + sqrt(z_0/norm(vb(g))) (dot(vb(p))_c - dot(vb(p))_(c,d))
$
其中, $T_(c phi.alt)$为规划的接触阶段时间, $z_0$ 为运动高度, $vb(p)_"h,i"$ 为hip 关节的位置

使用 PD 控制器追踪摆动腿足部在笛卡尔坐标系下的位置

在 _Robot Dynamics Lecture Notes_ 的3.9.3节中给出了末端执行器运动控制的关节力矩计算公式 (3.94)
$
  vb(tau)^star = vb(J)^T vb(Lambda)_e ( dot(vb(w))_e^star - dot(vb(J))_i dot(vb(q))_i) + vb(b) + vb(g)
$

文献中计算前馈控制力(仅根据期望加速度和动力学量计算)公式为
$
  vb(tau)_"ff,i" = vb(J)_i^T vb(Lambda)_i (bl(vb(a), fb)_"ref,i" - dot(vb(J))_i dot(vb(q))_i) + vb(C)_i dot(vb(q))_i + vb(G)_i
$
再添加关于位置和速度的PD反馈,得到总的控制器为
$
  vb(tau)_i = vb(J)_i^T [vb(K)_p (bl(vb(p)_"ref,i", fb) - bl(vb(p)_i, fb)) + vb(K)_d (bl(vb(v)_"ref,i", fb) - bl(vb(v)_i, fb))] + vb(tau)_"ff,i"
$
其中, $vb(K)_p$ 和 $vb(K)_d$ 是对角矩阵, 对角线上为比例和导数增益, 为了保证摆动腿的PD控制器保持稳定, 各个增益需要进行缩放
$
  K_(p,j) = omega_"des"^2 Lambda_(j j)
$
其中, $K_(p,j)$ 为增益矩阵 $vb(K)_p$ 第 $j$ 个对角元素, $omega_"des"$ 为期望的自然频率, $Lambda_(j j)$ 为工作空间惯性矩阵的第 $j$ 个对角元素

Virtual Predictive Support Polygon
暂略

斜坡地形的姿态调整

为了在没有视觉的情况下跨越阶梯和斜坡, 用每个足部位置 $vb(p)_i$ 来近似行走表面的局部斜率并调整期望位姿. 行走表面定义为一个平面
$
  z(x,y) = a_0 + a_1 x + a_2 y
$
其中的系数 $vb(a) = (a_0,a_1,a_2)^T$ 可通过求解一个最小二乘问题得到
$
  vb(a) = (vb(W)^T vb(W))^+ vb(W)^T vb(p)^z quad vb(W) = mat(vb(1), vb(p)^x, vb(p)^y)_(4 times 3) quad vb(p)^i = mat(p_1^i, p_2^i, p_3^i, p_4^i)
$

状态估计

使用一个两阶段传感器融合算法, 先估计方向, 再估计位置和速度. 第一阶段状态估计对IMU陀螺仪和加速度计的数据使用了一个方向滤波器, 核心思想为陀螺仪提供了本体角速度, 加速度计提供的重力加速度方向为本体倾斜和滚转方向做一个校正

其他内容暂略
