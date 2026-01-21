#import "@preview/physica:0.9.7": *
#set math.equation(numbering: "(1)")

= LIPM & Bipedal walking
参考资料:
- [1] Biped Walking Pattern Generation by using Preview Control of Zero-Moment Point
- [2] Capture Point: A Step toward Humanoid Push Recovery

#image("/content/Papers/LIPM/asset/pendulum_under_constraint.png", width: 30%)
当我们对一个倒立摆施加约束控制, 使其质心只能沿着一个任意给定的平面运动时, 我们便得到了一个简化的线性动力学模型, 称为三维线性倒立摆, 根据上图所示的坐标系, 约束平面可由法向向量$(k_x,k_y,1)$及其和z轴的交点$z_c$确定
$
  z = k_x x + k_y y + z_c
$
这里仅考虑约束平面是水平的,则在约束控制下的动力学方程为
$
  f vec(x, y, z_c) = vec(0, 0, m g) => f = m g / z_c \
  m dot.double(x) = f x + tau_y / z_c => dot.double(x) = g / z_c x + 1/(m z_c) tau_y \
  m dot.double(y) = f y - tau_x / z_c => dot.double(y) = g/z_c y - 1/(m z_c) tau_x
$
式中, $f$ 为连杆方向伸缩关节对质心的作用力, $tau$ 为支点处绕轴的力矩, 这里将其分解为作用在支点和质心处的力偶, 注意力矩的方向, 根据上式, 倒立摆质心的高度 $z$ 不会影响水平运动

对于该模型, 外力为支点处力矩及作用在质心上的重力, 因此其零力矩点 (ZMP) 为
$
  p_x = - tau_y /(m g) quad p_y = tau_x /(m g)
$
代入动力学方程得到
$
  dot.double(x) = g/z_c (x - p_x) quad dot.double(y) = g/z_c (y - p_y)
$

忽略支点处力矩并考虑二维情况, 动力学方程简化为
$
  dot.double(x) = g/z_c x
$
该动力学方程可以表示一个质量-弹簧系统, 对应的总能量称为"线性倒立摆轨道能量"
$
  E_"LIP" = 1/2 dot(x)^2 - g/(2z_c) x^2
$
如果质心朝向支点移动, 且有$E_"LIP">0$, 则有足够的能量使得质心越过支点继续运动, 否则质心将会在到达支点位置前停止并反向运动, 如果有$E_"LIP"=0$, 则质心将会正好停留在支点上方, 平衡状态对应系统的两个特征向量
$
  dot(x) = plus.minus x sqrt(g/z_c)
$
对于双足机器人, 线性倒立摆的轨道能量在摆动腿落地之前保持恒定, 不考虑交换支撑腿过程中的能量损耗 (质心速度不变), 可以通过期望的轨道能量来计算落脚位置, 在文献中, 为了计算 Capture Point, 期望的轨道能量为零, 对应系统稳定的特征向量
$
  x_"capture" = dot(x) sqrt(z_0/g)
$
