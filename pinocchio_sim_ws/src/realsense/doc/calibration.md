# Realsense D455 参数标定记录
标定工具：ros-noetic-calibration + MATLAB calibration toolbox

分辨率：720p，棋盘格7*6 间距108mm

内参矩阵：
$$
\begin{pmatrix}
     f_x &0&c_x\\
     0&f_y&c_y\\
     0&0&1
\end{pmatrix}=
\begin{pmatrix}
     644.591547364499&0&633.421613500672\\
     0&644.140670873576&372.042174330502\\
        0&0&1
\end{pmatrix}
$$
realsense D455出厂时标定内参矩阵
$$
\begin{pmatrix}
     f_x &0&c_x\\
     0&f_y&c_y\\
     0&0&1
\end{pmatrix}=
\begin{pmatrix}
     639.61022949&0&631.83618164\\
     0&638.98205566&369.17233276\\
        0&0&1
\end{pmatrix}
$$
比较接近，出厂内参接近真实值。
畸变参数：
$$
dist = \begin{pmatrix}
     -0.0434399514190001\\
     0.0416622489831636\\
     0
\end{pmatrix}
$$

## 世界坐标系、相机坐标系与像素坐标系间的关系

### 像素坐标系

![img](https://img-blog.csdnimg.cn/20210127172631244.png)

像素坐标系表示在图像中像素点的位置，如使用1280$\times$720分辨率，则u取值为0～1280，v取值为0~720，从mediapipe中可以直接得到归一化的像素坐标系坐标，再乘以对应的增益即可得像素坐标。

### 影像坐标系

影像坐标系是一个二维坐标系，是在影像内建立的坐标系，描述像素点在影像中的位置，分为以像素为单位的坐标系以及以物理尺寸为单位的坐标系。

影像坐标系与像素坐标系中的转化：
$$
\begin{bmatrix}
u\\v\\1
\end{bmatrix}=
\begin{bmatrix}
\frac{1}{dx}&0&u_0\\
0&\frac{1}{dy}&v_0\\
0&0&1
\end{bmatrix} 
\begin{bmatrix}
x\\y\\1
\end{bmatrix}
$$

### 相机坐标系

![img](https://img-blog.csdnimg.cn/20210128092438296.png#pic_center)

为了方便相机坐标系与影像坐标系间的转换，相机坐标系的原点建立在相机的中心，XY轴和影像xy坐标系的xy轴平行，Z轴垂直于像平面且朝向像平面，Z轴和像平面的交点正是影像xy坐标系的原点。

像平面上的所有像素点在相机坐标系下的Z坐标等于焦距f（小孔成像模型），因此若像素点在影像xy坐标系下的坐标为(x,y)，其在相机坐标系下的坐标为(x,y,f)。

假设像素点p是空间点P的投影点，坐标为：$(X_C,Y_C,Z_c)$

两点若在同一条直线上，则：
$$
\frac{x}{X_C}=\frac{y}{Y_C}=\frac{f}{Z_c}
$$
即：
$$
\begin{bmatrix}
u\\v\\1
\end{bmatrix}=
\begin{bmatrix}
\frac{1}{dx}&0&u_0\\
0&\frac{1}{dy}&v_0\\
0&0&1
\end{bmatrix} 
\begin{bmatrix}
\frac{f}{Z_c}&0&u_0\\
0&\frac{f}{Z_c}&v_0\\
0&0&\frac{1}{Z_c}
\end{bmatrix}
\begin{bmatrix}
X_C\\Y_C\\Z_C
\end{bmatrix}=\frac{1}{Z_C}\begin{bmatrix}
\frac{f}{dx}&0&u_0\\
0&\frac{f}{dy}&v_0\\
0&0&1
\end{bmatrix} \begin{bmatrix}
X_C\\Y_C\\Z_C
\end{bmatrix}
$$


### 世界坐标系
等待标定中...