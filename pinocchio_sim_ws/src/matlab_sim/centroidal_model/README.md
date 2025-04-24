# Centroidal NMPC

### Assumption

人的状态一般来说我们是无法完全观测的，我们不提倡采用过多的传感器获取人的信息，比如在人的鞋子处装鞋垫去感知人脚底处的力，这样会非常的繁琐。我们也很难去预测人的com的期望轨迹。在这里，我们做出假设：人脚底的力负责平衡人身体的重力，人的com的期望x,y位置由人的脚的位置给出- 单脚时为单脚的位置，双脚时为双脚位置相加之和。z的期望位置由人的身高进行估计。

这样的粗略的假设肯定会给控制带来问题，但好在我们有柔顺去弥补感知的不足。

用王总的话说：剩下的事情交给compliance~

### state:

$$
\begin{bmatrix} \theta &p 
&\omega & v & r_i\end{bmatrix}^{\top}
$$

$\theta$ ZYX 欧拉角  $p$ world frame下com的位置 $\omega$ 本体系下的com角速度  $v$ 本体系下com的线速度 $r_i$ 本体系下接触点的位置 $m$ 人的质量  $I$ 人的转动惯量矩阵（本体系下COM定义）
$$
\begin{bmatrix} \dot\theta \\\dot p \\ \dot\omega \\\dot v \\ \dot p_i\end{bmatrix} = \begin{bmatrix} T(\theta)\omega \\ R(\theta)p \\ I^{-1}(-\omega \times I \omega) + \sum f_i \times r_i  +  \sum f_s \times r_s \\ \frac {1}{m}\sum (f_i +f_s)-g \\ \dot p_i\end{bmatrix}
$$


### input:

$$
\begin{bmatrix} f_1 & f_2 & \dot p_1 & \dot p_2\end{bmatrix}^{\top}
$$

$f_1, f_2$ 双臂的两个接触点给人的力, $p_1, p_2$双臂两个接触点的位置（都定义在body frame）

### cost: 

我们的第一个控制目标是让人的质心始终跟踪我们给定的com轨迹：
$$
\epsilon_1 = \begin{bmatrix} \log(R_B R_{B,ref}^{\top})^{\
\wedge} \\ p - p_{ref} \\ \omega - \omega_{ref} \\ v - v_{ref}\end{bmatrix}
$$
第二个控制目标：我们希望力尽可能平滑 力与上一时刻求解力差值

第三个控制目标： 让接触位置不远离期望接触位置

### constraint：

动力学约束：



输入约束：上下界



接触约束：满足空间直线方程



力约束：满足摩擦锥约束

