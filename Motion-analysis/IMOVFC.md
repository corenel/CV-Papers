# Identification of a Moving Object's Velocity with a Fixed Camera

## Abstract
> In this paper, a continuous estimator strategy is utilized to asymptotically identify the six degree-of-freedom velocity of a moving object using a single fixed camera. The design of the estimator is facilitated by the fusion of homography-based techniques with Lyapunov design methods. Similar to the stereo vision paradigm, the proposed estimator utilizes different views of the object from a single camera to calculate 3D information from 2D images. In contrast to some of the previous work in this area, no explicit model is used to describe the movement of the object; rather, the estimator is constructed based on bounds on the object’s velocity, acceleration, and jerk.

本文提出了一种连续估计策略，用来在使用单目固定相机的情况下，渐进识别移动物体六个自由度上的速度。这种估计器的设计融合了基于单应性的技术（homography-based techniques）以及李雅普诺夫设计方法（Lyapunov design methods）。与立体视觉的模式相似，这种估计器利用单目相机对物体不同角度的二维图像，来计算三维信息。与本领域之前的一些工作不同的是，估计时没有物体运动的显式模型，而是基于物体运动的速度、加速度以及加速度的导数来构建估计器。

## 1 Introduction
> Often in an engineering application, one is tempted to use a camera to determine the velocity of a moving object. However, as stated in [8], the use of a camera requires one to interpret the motion of a 3-dimensional (3D) object through 2D images provided by the camera. That is, the primary problem is 3D information is compressed or nonlinearly transformed into 2D information; hence, techniques or methods must be developed to obtain 3D information despite the fact that only 2D information is available. 

工程应用上常常需要使用一个照相机来得到一个运动物体的速度。然而正如在[8]中所提到的那样，要做到这样，首先需要从相机提供的二维图像中描述三维物体的运动情况。也就是说，基本问题就是从三维信息到二维信息的转换是有压缩或者说是非线性的；因此，我们需要研究一种在只能获得二维信息的情况下，重建三维信息的技术或者说是方法。

> To address the identification of the object’s velocity (i.e., the motion parameters), many researchers have developed various approaches. For example, if a model for the object’s motion is known, an observer can be used to estimate the object’s velocity [10]. In [20], a window position predictor for object tracking was utilized. In [12], an observer for estimating the object velocity was utilized; however, a description of the object’s kinematics must be known. In [9], the problem of identifying the motion and shape parameters of a planar object undergoing Riccati motion was examined in great detail. In [13], an autoregressive discretetime model is used to predict the location of features of a moving object. In [1], trajectory filtering and prediction techniques are utilized to track a moving object. Some of the work [24] involves the use of camera-centered models that compute values for the motion parameters at each new frame to produce the motion of the object. In [2] and [21], object-centered models are utilized to estimate the translation and the center of rotation of the object. In [25], the motion parameters of an object are determined via a stereo vision approach.

为了应对识别物体速度（也就是物体的运动参量）这个问题，很多研究者探索了各种各样的方法。比如说，如果物体的运动模型是已知的，一个观测器（observer） 就能被用来估计物体速度（见[10]）。在[20]中，利用了一个窗口位置预测器（A window position predictor）来跟踪目标。在[12]中，在物体运动学描述已知的情况下，利用了一个观测器来估计物体的运动速度。在[9]中，对于作黎卡提运动（Riccati motion）的平面物体的运动以及形状参量的识别问题，做了详尽的讨论。在[13]中，自回归离散模型（an autoregressive discretetime model）被用来运动物体的特征点位置。在[1中，弹道滤波与预测技术（trajectory filtering and prediction technique）被用来追踪运动物体。一些工作（如[24]）涉及到使用相机为中心的模型（camera-centered models），计算每一帧上的运动参数的值，来描述物体运动（？）。在[2]和[21]中，利用以物体为中心的模型（object-centered models）来估计变换以及物体的旋转中心。在[25]中，使用了立体视觉的方法来确定物体的运动参数。

> While it is difficult to make broad statements concerning much of the previous work on velocity identification, it does seem that a good amount of effort has been focused on developing system theory-based algorithms to estimate the object’s velocity or compensate for the object’s velocity as part of a feedforward control scheme. For example, one might assume that object kinematics can be described as follows
$$
\dot x = Y(x) \phi
$$

> where $$x(t)$$, $$\dot x(t)$$ denote the object’s position vector and object’s velocity vector, respectively, $$Y(x)$$ denotes a known regression matrix, and $$\phi$$ denotes an unknown, constant vector. 

考虑到有了这么多关于速度识别的先前的工作，想要再做出更为广泛的陈述是很困难的。但是可以看出有很多人都把关注点转向了研究有系统的理论支持的算法来估计物体速度，或是通过前馈控制思想来补偿物体速度。例如，假设物体的运动学特征可以用如下表述：

$$
\dot x = Y(x) \phi
$$

其中，$$x(t)$$与$$\dot x(t)$$分别代表物体的位置向量与速度向量， $$Y(x)$$代表一个已知的回归矩阵，$$\phi$$代表一个未知的常数向量。

> As illustrated in [11], the object model of (1) can be used to describe many types of object models (e.g., constant-velocity, and cyclic motions). If $$x(t)$$ is measureable, it is easy to imagine how adaptive control techniques [22] can be utilized to formulate an adaptive update law that could compensate for unknown effects represented by the parameter $$\phi$$ for a typical control problem. In addition, if $$x(t)$$ is persistently exciting [22], one might be able to also show that the unknown parameter $$\phi$$ could be identified asymptotically. In a similar manner, robust control strategies or learning control strategies could be used to compensate for unknown object kinematics under the standard assumptions for these types of controllers (e.g., see [17] and [18]).

正如在[11]中提到的那样，(1)中的物体模型能够被用来描述许多运动（比如说匀速运动或者周期运动）。如果$$x(t)$$是可测的话，可以想见的是，自适应控制技术能够被用来制定一种自适应律，用以补偿在典型的控制问题中参量$$\phi$$带来的未知的影响。如果$$x(t)$$是持续激励，那么可以说未知参量$$\phi$$是可以被渐进地识别出来的。同样地，在[17]与[18]中可以看到，在这些类型的控制器的基本假设下，鲁棒控制策略与学习控制策略能够被用来补偿位置物体的运动学模型。

> While the above control techniques provide different methods for compensating for unknown object kinematics, these methods do not seem to provide much help with regard to identifying the object’s velocity if not much is known about the motion of the object. That is, from a systems theory point of view, one must develop a method of asymptotically identifying a time-varying signal with as little information as possible. This problem is made even more difficult because the sensor being used to gather the information about the object is a camera, and as mentioned before, the use of a camera requires one to interpret the motion of a 3D object from 2D images. To attack this double-goaded problem, we fuse homography-based techniques with a Lyapunov synthesized estimator to asymptotically identify the object’s unknown velocity 1 . Similar to the stereo vision paradigm, the proposed approach uses different views of the object from a single camera to calculate 3D information from 2D images. The homography-based techniques are based on fixed camera work presented in [3] which relies on the camera-in-hand work presented in [14]. The continuous, Lyapunov-based estimation strategy has its roots in an example developed in [19] and the general framework developed in [26]. The only requirements on the object are that its velocity, acceleration, and jerk be bounded, and that a single geometric length between two feature points on the object be known a priori.

虽然上述的控制方法提供了不同的方法来补偿未知物体的运动学模型，但是这些方法在不是很清楚物体运动模型的情况下，对于识别物体运动速度并没有太大的作用。也就是说，从系统论的角度来看，必须开发一种渐近方法以尽可能少的信息来确定时变信号。这个问题相较来说更为困难，因为用来收集有关对象的信息的传感器是一个照相机。正如前文所述，使用相机就需要从二维图像中解释一个3D对象的运动。为了解决这个问题，我们融合了基于单应性的技术以及李雅普诺夫设计方法。与立体视觉的模式相似，这种估计器利用单目相机对物体不同角度的二维图像，来计算三维信息。这种基于单应性的技术基于[3]中固定相机上的工作，而这工作又依赖于[14]中眼在手（camera-in-hand）上的工作。这种连续的基于李雅普诺夫的估计策略基于[19]中的一个例子以及[26]中的总体框架。这种方法对于物体的唯一要求是其速度、加速度以及加速度的导数是有界的，以及物体上两个特征点之间的几何距离是已知的。

## 2 Geometric Model
> To facilitate the subsequent object velocity identification problem, four target points located on an object denoted by $$O_i,\forall i=1,2,3,4 $$ are considered to be coplanar and not colinear. Based on this assumption, consider a fixed plane, denoted by $$\pi ^*$$, that is defined by a reference image of the object. In addition, let $$\pi$$ represent the motion of the plane containing the object feature points (see Figure 1). To develop a relationship between the planes, an inertial coordinate system, denoted by $$I$$, is defined where the origin coincides with the center of a fixed camera. The 3D coordinates of the target points on $$\pi$$ and $$\pi ^*$$ can be respectively expressed in terms of $$I$$

> $$
\overline m_i(t)=[x_i(t)\ y_i(t)\ z_i(t)]^T \\
\overline  m_i^*=[x_i\ y_i\ z_i]^T
$$

> under the standard assumption that the distances from the
origin of I to the target points remains positive (i.e., $$z_i (t),z^* _i\gt \epsilon$$ where $$\epsilon$$ is an arbitrarily small positive constant).

为了方便之后的物体速度识别问题，考虑物体上的4个特征点$$O_i,\forall i=1,2,3,4 $$是共面但是不共线的。在这个假设的基础上，考虑一个固定的平面$$\pi ^*$$，这个平面由物体的一个参考图象所定义。另外，令$$\pi$$表示包含着物体特征点的平面的运动（见图1）。为了探究平面之间的关系，定义一个原点在固定相机中心的惯性坐标系$$I$$。平面$$\pi$$与$$\pi ^*$$上的特征点的三维坐标相对于$$I$$可以表示如下：

$$
\overline m_i(t)=[x_i(t)\ y_i(t)\ z_i(t)]^T \\
\overline  m_i^*=[x_i\ y_i\ z_i]^T
$$

根据基本假设，从原点到特征点之间的距离始终是正的（也就是说，对于任意小的正常数$$\epsilon$$，$$z_i (t),z^* _i\gt \epsilon$$）。

> Orthogonal coordinate systems $$F$$ and $$F^*$$ are attached to $$\pi$$ and $$\pi ^*$$ , respectively, where the origin of the coordinate systems coincides with the object (see Figure 1). To relate the coordinate systems, let $$R(t),R^* \in SO(3)$$ denote the rotation between $$F$$ and $$I$$, and $$F^*$$ and $$I$$, respectively, and let $$x_f(t),x_f^* \in R^3$$ denote the respective translation vectors expressed in the coordinates of $$I$$. 

定义与$$\pi$$和$$\pi ^*$$相关联的正交坐标系$$F$$以及$$F^*$$，其坐标原点与物体相一致（见图1）。定义

> As also illustrated in Figure 1, $$n^* \in R^3$$ denotes the constant normal to the plane $$\pi ^*$$ expressed in the coordinates of $$I$$, $$s_i \in R^3$$ denotes the constant coordinates of the target points located on the object reference frame, and the constant distance $$d^* \in R
$$ from $$I$$ to $$F^*$$ along the unit normal is given by

> $$
d^* ={n^*}^T \overline m_i^*
$$

> The subsequent development is based on the assumption that the constant coordinates of one target point s i is known. For simplicity and without loss of generality, we assume that the coordinate s 1 is known (i.e., the subsequent development requires a single geometric length between two feature points on the object be known a priori).

> From the geometry between the coordinate frames depicted in Figure 1, the following relationships can be developed

> $$
\overline m_i = x_f + R_{s_i} \\
\overline m_i^* = x_f^* + R_{s_i}^*
$$

> After solving (6) for $$s_i$$ and then substituting the resulting expression into (5), the following relationships can be obtained

> $$
\overline m_i = \overline x_f + \overline R \overline m^*
$$


> where $$\overline R(t)\in SO(3)$$ and $$\overline x_f(t)\in R^3$$ are new rotational
and translational variables, respectively, defined as follows

> $$
\overline R = R(R^*)^T \\
\overline x_f = x_f - \overline R x^*_f
$$

> From (4), it is easy to see how the relationship in (7) can now be expressed as follows

> $$
\overline m_i = \left({\overline R + \frac{\overline x_f}{d^*}{n^*}^T}\right)\overline m_i^*
$$

> **Remark 1** The subsequent development requires that the constant rotation matrix $$R^*$$ be known. This is considered to be a mild assumption since the constant rotation matrix $$R^*$$ can be obtained a priori using various methods (e.g., a second camera, Euclidean measurements, etc.).

## 3 Euclidean Reconstruction
> The relationship given by (9) provides a means for formulating a translation and rotation error between $$F$$ and $$F^*$$ . Since the Euclidean position of $$F$$ and $$F^*$$ cannot be directly measured, a method for calculating the position and rotational error using pixel information is developed in this section (i.e., pixel information is the measurable quantity as opposed to \overline $$m_i(t)$$ and \overline $$m^*_i$$). To this end, the normalized 3D task-space coordinates of the points on $$\pi$$ and $$\pi^*$$ can be respectively expressed in terms of $$I$$ as $$m_i(t),m^*_i \in R^3$$ , as follows

> $$
m_i = \frac{\overline m_i}{z_i}=[\frac{x_i}{z_i},\frac{y_i}{z_i},1]^T \\
m_i^* = \frac{\overline m_i^*}{z_i^*}=[\frac{x_i^*}{z_i^*},\frac{y_i^*}{z_i^*},1]^T
$$

> The rotation and translation between the normalized coordinates can now be related through a Euclidean homography, denoted by $$H(t) \in R^{3\times 3}$$ , as follows

> $$
m_i=\underbrace{\frac{z^*_i}{z^i}}_{\alpha _i}\underbrace{(\overline{R}+\overline x_h(n^*)^T)}_H m^*_i
$$

> where $$\alpha _i \in R$$ denotes a depth ratio, and $$\overline x_h \in R^3$$ denotes a scaled translation vector that is defined as follows

> $$
\overline x_h = \frac{\overline x_f}{d^*}
$$

> In addition to having a task-space coordinate as described previously, each target point $$O_i$$ , $$O^*_i$$ will also have a projected pixel coordinate expressed in terms of $$I$$, denoted by $$u_i(t), v_i(t), u^*_i, v^*_i \in R$$, that is respectively defined as an element of $$p_i(t),p^*)i \in R^3$$ , as follows

> $$
p_i = [u_i, v_i, 1] ^ T \\
p^*_i=[u^*_i\ v^*_i\ 1]^T
$$

> The projected pixel coordinates of the target points are related to the normalized task-space coordinates by the following pin-hole lens models [6]

> $$
p_i=Am_i \\
p_i^*=Am_i^*
$$

> where $A\in R^{3\times 3}$ is a known, constant, and invertible intrinsic camera calibration matrix. After substituting (15) into (12), the following relationship can be developed

> $$
p_i = \alpha _i \underbrace{AHA^{-1}}_G p^*_i
$$

> where $$G(t) = [g_{ij}(t)] \forall i,j = 1,2,3 \in R^{3×3}$$ denotes a projective homography.After normalizing $$G(t)$$ by dividing through by the element $$g_{33}(t)$$, which is assumed to be nonzero without loss of generality, the projective relationship in (16) can be expressed as follows

> $$
p_i = \alpha _i g_{33} G_n p_i^*
$$

> where $$G_n(t) \in R^{3×3}$$ denotes the normalized projective homography. From (17), a set of 12 linear equations given by the 4 target point pairs ($$p^*_i ,p_i(t)$$) with 3 equations per target pair can be used to determine $$G_n(t)$$ and $$\alpha _i(t) g_{33}(t)$$. Based on the fact that the intrinsic calibration matrix $$A$$ is assumed to be known, (16) and (17) can be used to determine the product $$g_{33}(t) H(t)$$. By utilizing various techniques (e.g., see [7, 27]), the the product $$g_{33}(t) H(t)$$ can be decomposed into rotational and translational components as in (12). Specifically, the scale factor $$g_{33}(t)$$, the rotation matrix $$\overline R(t)$$, the unit normal vector $$n^*$$ , and the scaled translation vector denoted by $$\overline x_h (t)$$ can all be computed from the decomposition of the product $$g_{33}(t) H(t)$$. Since the product $$\alpha _i(t) g_{33}(t)$$ can be computed from (17), and $$g_{33}(t)$$ can be determined through the decomposition of the product $$g_{33}(t) H(t)$$, the depth ratio $$\alpha _i(t)$$ can be also be computed. Based on the assumption that $$R^*$$ is known and the fact that $$\overline R(t)$$ can be computed from the homography decomposition, (8) can be used to compute $$R(t)$$. Hence, $$R(t), \overline R(t), \overline x_h(t), n^*$$ , and the depth ratio $$\alpha _i(t)$$ are all known signals that can used in the subsequent estimator design.

## 4 Object Kinematics
> Based on information obtained from the Euclidean reconstruction, the object kinematics are developed in this section. To develop the translation kinematics for the object, $$e_v(t) \in R^3$$ is defined to quantify the translation of $$F$$ with respect to the fixed coordinate system $$F^*$$ as follows

> $$
e_v = p_e - p_e^*
$$

> In (18), $$p_e(t) \in R^3$$ denotes the following extended image coordinates [14] of an image point 3 on $$\pi$$ in terms of the inertial coordinate system $$I$$

> $$
p_e = [u_1, v_1, ln(z_1)]^T
$$

> where $$ln(\cdot )$$ denotes the natural logarithm, and $$p^*_e \in R^3$$  denotes the following extended image coordinates of the corresponding image point on $$\pi ^*$$ in terms of $$I$$

> $$
p_e^* = [u_1^*, v_1^*, ln(z_1^*)]^T
$$

> The first two elements of $$e_v (t)$$ are directly measured from the images. By exploiting standard properties of the natural logarithm, it is clear that the third element of $$e_v (t)$$ is equivalent to $$ln(\alpha _1)$$; hence, $$e_v (t)$$ is a known signal since $$\alpha _1 (t)$$ is computed during the Euclidean reconstruction. After taking the time derivative of (18), the following translational kinematics can be obtained (details available upon request)

> $$
\dot e_v = \dot p_e = \frac{\alpha _1}{z^*_1} A_e L_v \left[v_e - R[s_1]_{\times} R^T \omega _e \right]
$$

> where $$v_e (t), \omega _e (t) \in R^3$$ denote the unknown linear and angular velocity of the object expressed in $$I$$, respectively. In (21), $$A_e \in R^{3\times 3}$$ is defined as follows

> $$
A_e = A - 
\begin{bmatrix}
0 & 0 & u_0 \\
0 & 0 & v_0 \\
0 & 0 & 0
\end{bmatrix}
$$

> where $$u_0 , v_0 \in R$$ denote the pixel coordinates of the principal point (i.e., the image center that is defined as the frame buffer coordinates of the intersection of the optical axis with the image plane), and the auxiliary Jacobian-like matrix $$L_v (t) \in R^{3×3}$$is defined as

> $$
L_v = 
\begin{bmatrix}
1 & 0 & -\farc{x_1}{z_1} \\
0 & 1 & -\farc{y_1}{z_1} \\
0 & 0 & 1
\end{bmatrix}
$$

> To develop the rotation kinematics for the object, $$e_ \omega (t) \in R^3$$ is defined using the angle axis representation [23] to quantify the rotation of F with respect to the fixed coordinate system $$F^*$$ as follows

> $$
e_ \omega = u(t) \theta (t)
$$

> In (24), $$u(t) \in R^3 $$ represents a unit rotation axes, and $$\theta (t) \in R$$ denotes the rotation angle about u(t) that is assumed to be confined to the following region

> $$
-\pi \lt \theta (t) \lt \pi
$$

> After taking the time derivative of (24), the following expression can be obtained [4]

> $$
\dot e_\omega = L_\omega \omega _e
$$

> In (26), the Jacobian-like matrix $$L_ω (t) \in R^{3×3}$$ is defined as

> $$
L_\omega = I_3 - \frac{\theta}{2} [u]_{\times} + \left({1-\frac{sinc(\theta)}{sinc^2 \left({\frac{\theta}{2}}\right) }}\right) [u]_{\times}^2
$$

> where $$[u]_{\times}$$ denotes the $$3\times 3$$ skew-symmetric form of $$u(t)$$ and

> $$
sinc(\theta (t)) = \frac{sin\theta (t)}{\theta (t)}
$$

> **Remark 2** The structure of (18)-(20) is motivated by the fact that developing the object kinematics using partial pixel information clarifies the influence of the camera intrinsic calibration matrix. By observing the influence of the intrinsic calibration parameters, future efforts might be directed at developing an observer strategy that is robust to these parameters. Since the intrinsic calibration matrix is assumed to be known in this paper, the observer strategy could also be developed based solely on reconstructed Euclidean information (e.g., $$\overline x_h (t), \overline R(t)$$).

> **Remark 3** As stated in [23], the angle axis representation in (24) is not unique, in the sense that a rotation of $$− \theta(t)$$ about $$−u(t)$$ is equal to a rotation of $$\theta (t)$$ about $$u(t)$$. A particular solution for $$\theta(t)$$ and $$u(t)$$ can be determined as follows [23]

> $$
\theat _p = cos^{-1} \left(\frac{1}{2}(tr(\overline R)-1)\right) \\
[u_p]_{\times} = \frac{\overline R - \overline R ^T)}{2sin(\theta _p)}
$$

> where the notation $$tr(\cdot)$$ denotes the trace of a matrix and $$[u_p]_{\times}$$ denotes the $$3\times 3$$ skew-symmetric form of $$u_p (t)$$. From (28), it is clear that

> $$
0 \le \theta _p(t) \le \pi
$$

> While (29) is confined to a smaller region than $$\theta (t)$$ in (25), it is not more restrictive in the sense that

> $$
u_p \theat _p = u \theta
$$

> he constraint in (29) is consistent with the computation of $$[u(t)]_{\times}$$ in (28) since a clockwise rotation (i.e., $$−\pi \le \theta (t) le 0$$) is equivalent to a counterclockwise rotation (i.e., $$0 \le \theta (t) \le \pi$$) with the axis of rotation reversed. Hence, based on (30) and the functional structure of the object kinematics, the particular solutions $$\theta _p(t)$$ and $$u_p (t)$$ can be used in lieu of $$\theta (t)$$ and $$u(t)$$ without loss of generality and without confining $$\theta (t)$$ to a smaller region. Since, we do not distinguish between rotations that are off by multiples of $$2 \pi$$, all rotational possibilities are considered via the parameterization of (24) along with the computation of (28).

> **Remark 4** By exploiting the fact that $$u(t)$$ is a unit vector (i.e., $$||u||^2 = 1$$), the determinant of $$L_\omega (t)$$ can be derived as follows [16]

> $$
det L_\omega = \frac{1}{sinc^2 \left({\frac{\theta}{2}}\right)}
$$

> From (31), it is clear that $$L_\omega (t)$$ is only singular for multiples of $$2 \pi$$ (i.e., out of the assumed workspace).