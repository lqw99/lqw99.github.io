#import "@preview/physica:0.9.7": *
#let so3 = $italic("so")(3)$
#let SO3 = $italic("SO")(3)$
#let SE3 = $italic("SE")(3)$
#let fa = $cal(A)$
#let fb = $cal(B)$
#let fc = $cal(C)$
#let fi = $cal(I)$ // frame of inertial
#let fe = $cal(E)$ // frame of end-effector
#let bl(v1, v2) = $attach(v1, bl: v2)$

// numbering lv1 header
#set heading(numbering: "1.")
#let ct = counter(math.equation) // counter for equation
// #let ct = counter("eq")//
// #set math.equation(numbering: "(1.1)")
#set math.equation(numbering: it => {
  ct.display("(1.1)")
})
// set counter as n.1 when a lv1 header show up
#show heading.where(level: 1): it => {
  it
  ct.step()
  ct.step(level: 2)
}
#show math.equation.where(block: true): it => {
  it
  if it.numbering != none {
    if ct.get().len() == 2 {
      ct.step(level: 2)
    }
  }
}





#align(center)[
  Robot Dynamics Lecture Notes
]

#show outline.entry: it => link(
  it.element.location(),
  it.indented(it.prefix(), it.body()),
)

#outline()


Reference

- [1] #link("https://ethz.ch/content/dam/ethz/special-interest/mavt/robotics-n-intelligent-systems/rsl-dam/documents/RobotDynamics2017/RD_HS2017script.pdf")[Robot Dynamic Lecture Note]

// - [2] #link("https://arxiv.org/pdf/1711.02508")[Quaternion kinematics for the error-state Kalman filter]


= Introduction

robot can be described as a single body or a set of bodies with different forces applied. a robot arm moving in free space is driven by actuator forces on joints, a legged robot is additionally driven by contact forces on feet and a flying vehicle is driven by aerodynamic forces

category of robot:
- fixed base: robot arms which are rigidly bolted to ground, the dof of system is equal to the number of actuators
- floating base: mobile robots have moving base, the motion of floating base is influenced by external forces on system like contact or aerodynamic forces

a list of math notations

= Kinematics

description of the motion of points, bodies, and systems of bodies, only consider *how* to move but not *why*


== Position, rotation, and velocity
for a moving point, need a position vector $vb(r) in RR^3$ and its derivatives; for a rigid body, a rotation $vb(phi) in SO3$ is additionally needed


- position vector is position of point B relative to point A:
$
  vb(r)_(A B) = vb(p)_B - vb(p)_A
$
to numerically express the components of the position vector, need to define a reference frame with an orho-normal basis
$
  fa = {e_x^fa,e_y^fa,e_z^fa}
$
so position vector is given by
$
  vb(r)= x e_x^fa + y e_y^fa + z e_z^fa = mat(e_x^fa, e_y^fa, e_z^fa) vec(x, y, z) quad bl(vb(r), fa)= vec(x, y, z)
$

- velocity of point B relative to point A is given by $dot(vb(r))_(A B)$

- orientation of body fixed frame $fb$ w.r.t a reference frame $fa$ is given by $r_(fa fb)$, and it can be parametrized in several ways

for two frame with same origin point O, and a point P in space, the position vector $vb(r)_(O P)$ can be expressed in two frame:
$
  bl(vb(r)_(O P), fa) = vec(bl(r_x, fa), bl(r_y, fa), bl(r_z, fa)) quad bl(vb(r)_(O P), fb) = vec(bl(r_x, fb), bl(r_y, fb), bl(r_z, fb))
$

basis vectors of frame $fb$ can be expressed in frame $fa$ by
$
  fb = {e_x^fb,e_y^fb,e_z^fb} = {bl(e_x^fb, fa),bl(e_y^fb, fa),bl(e_z^fb, fa)}
$
then the relationship between two position vector is
$
  vb(r)_(O P) = mat(e_x^fb, e_y^fb, e_z^fb) vec(bl(r_x, fb), bl(r_y, fb), bl(r_z, fb)) => bl(vb(r)_(O P), fa) = mat(bl(e_x^fb, fa), bl(e_y^fb, fa), bl(e_z^fb, fa)) vec(bl(r_x, fb), bl(r_y, fb), bl(r_z, fb)) = vb(R)_(fa fb) dot bl(vb(r)_(O P), fb)
$

where $vb(R)_(fa fb)$ is rotation matrix and columns of it are orthogonal unit vector, so rotation matrix is orthogonal and belongs to special (comes from the det of matrix is 1 not -1) orthonormal group $SO3$

rotations have two different interpretations:
- passive rotation: a mapping between frames, $vb(R)_(fa fb)$ maps the same vector from frame $fb$ to frame $fa$ by $bl(vb(u), fa) = vb(R)_(fa fb) dot bl(vb(u), fb)$
- active rotation: an operator that rotates a vector to another vector in the same reference frame by $bl(vb(v), fa) = vb(R) dot bl(vb(u), fa)$, which is not used in robot dynamics

composition of rotations is given by
$
  fa ->^(vb(R)_(fa fb)) fb ->^(vb(R)_(fb fc)) fc => bl(vb(u), fa) = (vb(R)_(fa fb)vb(R)_(fb fc)) bl(vb(u), fc)
$

representation of rotation use parameter vector
- Euler angles
ZXZ Euler angles (propery Euler angles) $vb(R) = vb(R)_z (z_1) vb(R)_x (x) vb(R)_z (z_2)$

ZYX Euler angles (Tait-Bryan angles, yaw-pitch-roll) $vb(R) = vb(R)_z (z) vb(R)_y (y) vb(R)_x (x)$

XYZ Euler angles (Cardan angles) $vb(R) = vb(R)_x (x) vb(R)_y (y) vb(R)_z (z)$
- angle-axis: defined by an angle $theta$ and an axis $vb(n)$, combine this two parameters to obtain a rotation vector (Euler vector) as $vb(phi) = theta dot vb(n)$

- unit quaternions: use Hamiltion type, details see [2]

angular velocity $vb(omega)_(fa fb)$ is the limit of rotational motion of $fb$ w.r.t $fa$:
$
  vb(omega)_(fa fb) = lim_(epsilon->0) (vb(phi)_(fb(t) fb(t+epsilon)))/epsilon
$
relationship between angular velocity and rotation matrix is defined by
$
  [bl(vb(omega)_(fa fb), fa)]_times = dot(vb(R))_(fa fb) dot vb(R)_(fa fb)^T
$
angular velocity in different frame is expressed in
$
  bl(vb(omega)_(fa fb), fb) = vb(R)_(fb fa) dot bl(vb(omega)_(fa fb), fa)
$

the relationship between time derivatives of rotation parameterizations and angular velocity is given by [1] 2.5.1, used to update rotation parameter (orientation) with angular

== Velocity in a moving body

let $fa$ be the inertial fixed frame, $fb$ is the frame fixed on a moving body, we have:
- absolute velocity of point P on body is $vb(v)_P$
- absolute acceleration of point P on body is $vb(a)_P = dot(vb(v))_P$
- absolute angular velocity of body is $vb(Omega)_fb = vb(omega)_(fa fb)$
- absolute angular acceleration of body is $vb(Psi)_fb = dot(vb(Omega))_fb$



the time derivative of a position vector expressed in a frame $fc$ is not always equal to the time derivative of coordinates of a position in a frame $fc$, in other word,
$
  bl(dot(vb(r))_(A P), fc) = bl(vb(v)_(A P), fc)= bl((dv(, t) vb(r)_(A P)), fc) eq.not
  dv(, t) (bl(vb(r)_(A P), fc))
$
unless frame $fc$ is an inertial frame

position of point P is written as
$
  vb(r)_(A P) = vb(r)_(A B) + vb(r)_(B P) => bl(vb(r)_(A P), fa) = bl(vb(r)_(A B), fa) + vb(R)_(fa fb) dot bl(vb(r)_(B P), fb)
$

differentiating $bl(vb(r)_(A B), fa)$ w.r.t time results in
$
  dv(, t) (bl(vb(r)_(A P), fa)) = dv(, t) (bl(vb(r)_(A B), fa) ) + dv(, t) vb(R)_(fa fb) dot bl(vb(r)_(B P), fb) + vb(R)_(fa fb) dot dv(, t) (bl(vb(r)_(B P), fb))
$

point P is on body, the position vector $vb(r)_(B P)$ expressed in frame $fb$ is constant, frome the relationship between angular velocity and rotation matrix, $dot(vb(R))_(fa fb) = [bl(vb(omega)_(fa fb), fa)]_times dot vb(R)_(fa fb)$, yielding
$
  dv(, t) (bl(vb(r)_(A P), fa))& = dv(, t) (bl(vb(r)_(A B), fa) ) + [bl(vb(omega)_(fa fb), fa)]_times dot vb(R)_(fa fb) dot bl(vb(r)_(B P), fb) \
  & = dv(, t) (bl(vb(r)_(A B), fa) ) + [bl(vb(omega)_(fa fb), fa)]_times dot bl(vb(r)_(B P), fa)
$
which is rigid body velocity formulation, for a point P on rigid body B, the absolute velocity is
$
  vb(v)_P = vb(v)_B + Omega times vb(r)_(B P)
$
for acceleration, result is
$
  vb(a)_P = vb(a)_B + vb(Psi) times vb(r)_(B P) + vb(Omega) times (vb(Omega) times vb(r)_(B P))
$

vector differentiation in moving frame is given by Euler differentiation rule, to get  absolute velocity of P in frame $fb$ from position vector $vb(r)_(A P)$ in frame $fb$, we must transform $bl(vb(r)_(A P), fb)$ into frame $fa$, then differentiating it, then transform back to frame $fb$
$
  bl(vb(v)_P, fb) = vb(R)_(fb fa) dot dv(, t) (vb(R)_(fa fb) dot bl(vb(r)_(A P), fb)) = vb(R)_(fb fa) dot (vb(R)_(fa fb) dot dv(, t)bl(vb(r)_(A P), fb) + dot(vb(R))_(fa fb) dot bl(vb(r)_(A P), fb) ) \
  = dv(, t)bl(vb(r)_(A P), fb) + bl(vb(omega)_(fa fb), fb) times bl(vb(r)_(A P), fb)
$

== Kinematics of system of bodies

robotic system can be model as open kinematic structure composed of $n_l = n_j+1$ links connected by $n_j$ joints (prismatic or revolute, one dof), two connected bodies related by a transformation
$
  vb(T)_(fb_(i-1) fb_i) = vb(T)_(fb_(i-1) fb_i) (q_i) quad i = 1,dots,n_j
$

for fixed base system, frame attached to root link $fb_0$ is world fixed (inertial) frame

generalized coordinate vector is used to describe configuration of robot:
$
  vb(q) = vec(q_1, dots.v, q_n)
$
in most cases, each generalized coordinate corresponds to a dof, without additional kinematic constriants, minimal set of generalized coordinates is called minimal coordinates, the choice of generalized coordinates is not unique

configuration of end-effector is described by relative position and orientation w.r.t a reference frame (inertial or root frame)
$
  vec(vb(r)_e, vb(phi)_e) in SE3 =>^"parameterized" vb(chi)_e = vec(vb(chi)_"eP", vb(chi)_"eR") in RR^m
$

end-effector operates in operational space which depends on geometry and structure of robot, is described by
$
  vb(chi)_o = vec(vb(chi)_"oP", vb(chi)_"oR") in RR^(m_0)
$
which are independent coordinates, is a minimal selection of end-effector configuration and $m_0 <= n_j$

forward kinematics: mapping between generalized joint coordinates $vb(q)$ and end-effector configuration
$
  vb(chi)_e = vb(chi)_e (vb(q))
$

for a serial linkage system
$
  vb(T)_(fi fe)(vb(q)) = vb(T)_(fi 0) dot (product_(k=1)^(n_j) vb(T)_(k-1,k) (q_k)) dot vb(T)_(n_j fe) = mat(vb(R)_(fi fe), bl(vb(r)_(I E) (q), fi); vb(0)_(1 times 3), 1)
$
for fixed base robot, first frame of link 0 is not moving w.r.t inertial frame and end-effector is rigidly connected to last body, then $vb(T)_(fi 0)$ and $vb(T)_(n_j fe)$ are constant transformation

differential kinematics is given by linearization of forward kinematics:
$
  vb(chi)_e + delta vb(chi)_e = vb(chi)_e (vb(q)+delta vb(q)) = vb(chi)_e + pdv(vb(chi)_e (vb(q)), vb(q)) delta vb(q) + O(delta vb(q)^2) \
  => delta vb(chi)_e approx pdv(vb(chi)_e (vb(q)), vb(q)) delta vb(q) = vb(J)_(e A) (vb(q)) delta vb(q) quad vb(J)_(e A) (vb(q))_(i j) = pdv(chi_i, q_j)
$
$vb(J)_(e A)$ is a $m times n_j$ analytical Jacobian matrix, represents an approximation in context of finite differences and exact relation between generalized velocity and time-derivatives of end-effector configuration parameters:
$
  Delta vb(chi)_e approx vb(J)_(e A) (vb(q)) Delta vb(q) quad dot(vb(chi))_e = vb(J)_(e A) (vb(q)) dot(vb(q))
$

analytical Jacobian is divided into position and rotation Jacobian
$
  vb(J)_(e A) = vec(vb(J)_(e A_P), vb(J)_(e A_R)) = vec(pdv(vb(chi)_(e P), vb(q)), pdv(vb(chi)_(e R), vb(q)))
$

geometric Jacobian is a unique Jacobian than relates generalized velocity and velocity of end-effector
$
  vb(w)_e = vec(vb(v)_e, vb(omega)_e) = vb(J)_(e 0) (vb(q)) dot(vb(q)) quad vb(J)_(e 0) in RR^(6 times n_j)
$

the velocity of link k is given by
$
  dot(vb(r))_(I k) = dot(vb(r))_(I (k-1)) + vb(omega)_(fi (k-1)) times vb(r)_((k-1) k)
$
where $I$ is origin of frame $fi$, for numerical addition, all vectors are expressed in same coordinate system
#image("./assets/serial-linkage-arm.png", width: 60%)
denoting base (inertial) frame as 0 and end-effector frame as $n+1$, origin of frame $k$ (is the body frame of link k) is at joint k and the end-effector velocity is given by
$
  dot(vb(r))_(I E) = sum_(k=1)^n vb(omega)_(fi k) times vb(r)_(k(k+1))
$

frame of link k is aligned with joint k, then
$
  vb(omega)_((k-1)k) = vb(n)_k dot(q)_k => vb(omega)_(fi k) = sum_(i=1)^k vb(n)_i dot(q)_i
$
which results in
$
  dot(vb(r))_(I E) = sum_(k=1)^n (sum_(i=1)^k vb(n)_i dot(q)_i times vb(r)_(k(k+1))) = sum_(k=1)^n vb(n)_k dot(q)_k times vb(r)_(k(n+1))
$
then geometric Jacobian is given by
$
  dot(vb(r))_(I E) = underbrace(mat(vb(n)_1 times vb(r)_(1(n+1)), vb(n)_2 times vb(r)_(2(n+1)), dots.c, vb(n)_n times vb(r)_(n(n+1))), vb(J)_(e 0_P)) vec(dot(q)_1, dot(q)_2, dots.v, dot(q)_n) \
  vb(omega)_(fi fe) = underbrace(mat(vb(n)_1, vb(n)_2, dots.c, vb(n)_n), vb(J)_(e 0_R)) vec(dot(q)_1, dot(q)_2, dots.v, dot(q)_n)
$
we need to define this w.r.t a basis like $fi$, then
$
  bl(vb(J)_(e 0), fi) = vec(bl(vb(J)_(e 0_P), fi), bl(vb(J)_(e 0_R), fi)) = mat(bl(vb(n)_1, fi) times bl(vb(r), fi)_(1(n+1)), bl(vb(n), fi)_2 times bl(vb(r), fi)_(2(n+1)), dots.c, bl(vb(n), fi)_n times bl(vb(r), fi)_(n(n+1)); bl(vb(n), fi)_1, bl(vb(n), fi)_2, dots.c, bl(vb(n), fi)_n)
$
the rotation axis is given by
$
  bl(vb(n), fi)_k = vb(R)_(fi (k-1)) dot bl(vb(n)_k, (k-1))
$

analytical Jacobian matrix and geometric Jacobian matrix is related by
$
  dot(vb(chi))_e = vb(J)_(e A) (vb(q)) dot(vb(q)) quad vb(J)_(e A) in RR^(m_e times n_j) \
  vb(w)_e = vec(vb(v)_e, vb(omega)_e) = vb(J)_(e 0) (vb(q)) dot(vb(q)) quad vb(J)_(e 0) in RR^(6 times n_j) \
  vb(w)_e = vb(E)_e (vb(chi)_e) dot(vb(chi))_e quad vb(E)_e = mat(vb(E)_P, vb(0); vb(0), vb(E)_R) in RR^(6 times m_e)\
  => vb(J)_(e 0) (vb(q)) = vb(E)_e (vb(chi)_e) vb(J)_(e A)(vb(q))
$

== Kinematic Control Methods

inverse differential Kinematics

geometric Jacobian performs a mapping from joint space velocity to end-effector velocity, but in many cases, given a desired end-effector velocity, how to get corresponding joint velocity, one way is take the inverse or pseudo-inverse of Jacobian
$
  dot(vb(q)) = vb(J)_(e 0)^+ vb(w)_e^star
$
there are many kinds of pseudo-inverse methods and solution is not unique

when a configuration $vb(q)_s$ that the rank of Jacobian $vb(J)_(e 0)(vb(q)_s)$ is smaller than the number of operational space coordinates (number of controllable end-effector DOFs), the configuration is called singular. In a singular configuration, for a desired end-effector velocity $vb(w)_e^star$, there may not exist a corresponding generalized velocity $vb(dot(q))$, by taking Moore-Penrose pseudo inverse, the solution of $dot(vb(q)) = vb(J)_(e 0)^+ vb(w)_e^star$ minimizes the least square error $norm(vb(w)_e^star - vb(J)_(e 0) dot(vb(q)))^2$. However, small desired velocity in singular direction will lead to extremely high joint velocity, which is problematic for inverse kinematics algorithms

to deal with the bad condition of singular Jacobian, a common approach is to use damped version of Moore-Penrose pseudo-inverse
$
  dot(vb(q)) = vb(J)_(e 0)^T (vb(J)_(e 0) vb(J)_(e 0)^T - lambda^2 vb(I))^(-1) vb(w)_e^star
$
the larger damping parameter $lambda$, the more stable the solution, but the slower the convergence

when a configuration $vb(q)^star$ that the rank of Jacobian $vb(J)_(e 0)(vb(q)^star)$ is smaller than the number of joints $n$, the configuration is called redundant. the Moore-Penrose pseudo-inverse
$
  dot(vb(q)) = vb(J)_(e 0)^T (vb(J)_(e 0) vb(J)_(e 0)^T)^(-1) vb(w)_e^star
$
minimizes $norm(dot(vb(q)))^2$ while $vb(w)_e^star = vb(J)_(e 0) dot(vb(q))$, then redundancy means there are infinite solutions
$
  dot(vb(q)) = vb(J)_(e 0)^+ vb(w)_e^star + vb(N) dot(vb(q))_0
$
where $vb(N) = cal(N) (vb(J)_(e 0))$ is null-space projection matrix of $vb(J)_(e 0)$ fulfilling $vb(J)_(e 0) vb(N) = vb(0)$, the generalized velocity could be modified by any choice of $dot(vb(q))_0$ without changing end-effector velocity
$
  vb(w)_e^star = vb(J)_(e 0) dot(vb(q)) = vb(J)_(e 0) (vb(J)_(e 0)^+ vb(w)_e^star + vb(N) dot(vb(q))_0) = vb(w)_e^star quad forall dot(vb(q))_0
$

to determine the null-space projection matrix, the simplest way is given by
$
  vb(N) = vb(I) - vb(J)_(e 0)^+vb(J)_(e 0)
$


inverse kinematics is to find a joint configuration as a function of a given end-effector configuration
$
  vb(q) = vb(q) (vb(chi)_e^star)
$

analytical solutions

numerical solution: the differences in joint space coordinates can be mapped to differences in end-effector coordinates by analytical Jacobian
$
  Delta vb(chi)_e = vb(J)_(e A) Delta vb(q)
$

given a start configuration $vb(q)^0$ and a desired end-effector configuration $vb(chi)_e^star$, the inverse kinematics problem can be solved iteratively

trajectory control: to follow a predefined task-space trajectory, pure inverse differential kinematics is insufficient, a weighted tracking error (pose error) feedback need to be added

for position tracking with a given predefined position $vb(r)_e^star (t)$ and velocity $dot(vb(r))_e^star (t)$, the trajectory controller is
$
  dot(vb(q))^star = vb(J)_(e 0_P)^+ (vb(q)^t) (dot(vb(r))_e^star (t) + k_(p P) Delta vb(r)_e^t) quad Delta vb(r)_e^t = vb(r)_e^star (t) - vb(r)_e (vb(q)^t)
$

for orientation tracking with predefined orientations $vb(chi)_R^star (t)$ and angular velocity $vb(w)^star (t)$, selection of parameterization leads to different trajectories, best approach is to follow the shortest rotation approach
$
  dot(vb(q))^star = vb(J)_(e 0_R)^+(vb(q)^t) (dot(vb(w))_e^star (t) + k_(p R) Delta vb(phi)) quad vb(R)_(cal(G) cal(S)) (Delta vb(phi)) = vb(R)_(cal(G) fi) (vb(phi)^star) vb(R)_(cal(S) fi)^T (vb(phi)^t)
$

== Floating base kinematics

floating-based robot is described by $n_b$ unactuated base coordinates $vb(q)_b$ and $n_j$ actuated joint coordinates $vb(q)_j$, the unactuated base is free in translation and rotation
$
  vb(q)_b = vec(vb(q)_(b_P), vb(q)_(b_R)) in RR^3 times SO3
$

generalized velocity and acceleration vector (not time-derivatives of parameterization) is
$
  vb(u) = vec(bl(vb(v)_B, I), bl(vb(w)_(I B), B), dot(phi)_1, dots.v, dot(phi)_(n_j)) in RR^(6+n_j) quad dot(vb(u)) = vec(bl(vb(a)_B, I), bl(vb(psi)_(I B), B), dot.double(phi)_1, dots.v, dot.double(phi)_(n_j)) in RR^(6+n_j) \
  vb(u) = vb(E)_(f b) dot(vb(q)) quad vb(E)_(f b) = mat(vb(I)_(3 times 3), 0, 0; 0, vb(E)_(chi_R), 0; 0, 0, vb(I)_(n_j times n_j))
$
the position vector of a point Q which is fixed at the end of kinematic chain, w.r.t inertial frame is given by
$
  bl(vb(r)_(I Q)(vb(q)), fi) = bl(vb(r)_(I B)(vb(q)), fi) + vb(R)_(fi fb)(vb(q)) dot bl(vb(r)_(B Q)(vb(q)), fb)
$

time differentiation of position vector gives

$
  bl(vb(v)_Q, fi) = bl(vb(v)_(I B)(vb(q)), fi) + dot(vb(R))_(fi fb)(vb(q)) dot bl(vb(r)_(B Q)(vb(q)), fb) + vb(R)_(fi fb)(vb(q)) dot bl(dot(vb(r))_(B Q)(vb(q)), fb) \
  = mat(vb(I)_(3 times 3), - vb(R)_(fi fb) dot [bl(vb(r)_(B Q), fb)]_times, vb(R)_(fi fb) dot bl(vb(J)_(P_(q_j)), fb)(vb(q)_j)) dot vb(u) \
  bl(vb(omega)_(fi cal(Q)), fi) = bl(vb(omega)_(fi fb), fi) + bl(vb(omega)_(fb cal(Q)), fi) \
  = mat(vb(0)_(3 times 3), vb(R)_(fi fb), vb(R)_(fi fb) dot bl(vb(J)_(R_(q_j)), fb)(vb(q)_j)) dot vb(u)
$
then the mapping from generalized velocity $vb(u)$ to operational space twist of frame $cal(Q)$ is given by spatial Jacobian
$
  vec(bl(vb(v)_Q, fi), bl(vb(omega)_(fi cal(Q)), fi)) = bl(vb(J)_Q, fi) (vb(q)) dot vb(u)
$

contact can be modeled as kinematic constriants, every contact point imposes three constriants
$
  bl(vb(r)_(I C_i), fi) = "const" quad bl(dot(vb(r))_(I C_i), fi) = bl(dot.double(vb(r))_(I C_i), fi) = vec(0, 0, 0)
$
the velocity and acceleration constriants can be expressed as a function of generalized velocity and acceleration using contact point Jacobian (position part of spatial Jacobian)
$
  bl(vb(J)_(C_i), fi) dot vb(u) = 0 quad bl(vb(J)_(C_i), fi) dot dot(vb(u)) + bl(dot(vb(J))_(C_i), fi) dot vb(u) = vb(0)
$
for $n_c$ active contacts, contact Jacobians are stacked to
$
  vb(J)_c = vec(vb(J)_(C_1), dots.v, vb(J)_(C_n_c)) in RR^(3 n_c times n_n)
$
stacked Jacobian can be split into
$
  vb(J)_c = mat(vb(J)_(c,b), vb(J)_(c,j))
$
where $vb(J)_(c,b)$ is the relation between base motion and contact constriants, if the rank of it is full (6), base motion can be controlled from joint motion

for point contacts of quadruped, each point point feet imposes three independent constriants. In case of two point contacts, the rank of stacked contact Jacobian is 6 but the base part is only 5 (robot cannot change orientation around the line of support). In case of three point contacts, rank of stacked contact Jacobian is 9 and the base part is 6, which means base position and orientation is fully controllable by joints and there are three internal constriants

for extended feet of humanoid, additional constriants are required to limit foot rotation. a common way is to assign multiple contact points on same link

applying inverse kinematics to floating base system is to achieve task-space motion without violating contact constriants. a multi-task approach with priorization where contact constriants are considered to have higher priority

= Dynamic

== Classical Mechanics

Newton's second law to a particle with (infinitesimal) mass
$
  m dot.double(vb(r)) = vb(F) quad dd(m) dot.double(vb(r)) = dd(vb(F))
$

variation of a quantity describes: for a fixed instant in time, all possible (admissible) directions the quantity may move while adhering to constriants

variation of a position vector is called virtual displacement
$
  delta vb(r)(vb(q),t) = sum_(k=1)^n_q pdv(vb(r), q_k) delta q_k
$

for a single body, position, velocity, acceleration vector of a point mass on body is given by
$
  vb(r) = vb(r)_(O S) + rho \
  dot(vb(r)) = vb(v)_S + vb(Omega) times vb(rho) = mat(vb(I), -[vb(rho)]_times) vec(vb(v)_S, vb(Omega)) \
  dot.double(vb(r)) = vb(a)_S + vb(Psi) times vb(rho) + vb(Omega) times (vb(Omega) times vb(rho)) = mat(vb(I), -[vb(rho)]_times)vec(vb(a)_S, vb(Psi)) + [vb(Omega)]_times [vb(Omega)]_times vb(rho)
$
where $vb(r)_(O S)$ is absolute position of body point S, $vb(rho)$ is relative position of $dd(m)$ w.r.t S and $vb(v)_S,vb(Omega),vb(a)_S$ and $vb(Psi)$ are absolute velocities and accelerations of point S.

motion is described using generalized coordinates
$
  vec(vb(v)_S, vb(Omega)) = mat(vb(J)_P; vb(J)_R) dot(vb(q)) quad vec(vb(a)_S, vb(Psi)) = mat(vb(J)_P; vb(J)_R) dot.double(vb(q)) + mat(dot(vb(J))_P; dot(vb(J))_R) dot(vb(q))
$

virtual displacement of body element $dd(m)$ is expressed as
$
  delta vb(r) = delta vb(r)_S + vb(delta Phi) times vb(rho) = mat(vb(I), -[vb(rho)]_times) vec(delta vb(r)_S, vb(delta Phi))
$
where $vb(delta Phi)$ is the variation of infinitesimal rotation of local body-fixed frame defined at point S, the variation is not taken w.r.t an orientation quantity

for a multi-body system, the motion is described using generalized coordinates $vb(q)$, virtual displacements are
$
  vec(delta vb(r)_s, vb(delta Phi)) = mat(vb(J)_P; vb(J)_R) delta vb(q)
$

principle of virtual work + d'Alambert's principle
$
  delta W = integral_fb delta vb(r) dot (dot.double(vb(r)) dd(m) - dd(vb(F)_"ext")) = 0 quad forall delta vb(r)
$


== Newton-Euler method

use principle of virtual work to a single body, define $m = integral_fb dd(m)$ as body mass, $integral_fb vb(rho) dd(m) = 0$ as S is center of geometry, $vb(I)_S = integral_fb - [vb(rho)]_times^2 dd(m) = integral_fb [vb(rho)]_times [vb(rho)]_times^T dd(m)$ is inertia matrix/tensor around S, we get
$
  0 = delta W = vec(delta vb(r)_S, vb(delta Phi))^T (mat(m vb(I), vb(0); vb(0), vb(I)_S)vec(vb(a)_S, vb(Psi))+vec(vb(0), [vb(Omega)]_times vb(I)_S vb(Omega))-vec(vb(F)_"ext", vb(T)_"ext")) quad forall vec(delta vb(r)_S, vb(delta Phi))
$

definitions about linear and angular momentum and their changes are
$
  vb(p)_S = m vb(v)_S quad dot(vb(p))_S = m vb(a)_S \
  vb(N)_S = vb(I) dot vb(Omega) quad dot(vb(N))_S = vb(I)_S dot vb(Psi) + vb(Omega) times (vb(I)_S dot vb(Omega))
$
which results in Newton-Euler equation
$
  dot(vb(p))_S = vb(F)_("ext",S) \
  dot(vb(N))_S = vb(T)_("ext",S)
$
for numerical calculation, the terms of changes in momentum must be expressed in same coordinate system, and for inertia tensor, $bl(vb(I)_S, fb) = vb(R)_(fb fa) dot bl(vb(I)_S, fa) dot vb(R)_(fb fa)^T$

to use Newton-Euler method for multi-body system, an approach is to separate all bodies at joints and consider every body as a single body, then apply constriant forces as external forces, in a general 3D and fixed base case, there $n_j$ joints and $n_j$ free bodies, so a system consists of $6 n_j$ dofs and $5 n_j$ motion constriants

another approach is to describe motion as a function of generalized coordinates

== Lagrange method

three concepts:
- generalized coordinates $vb(q)$ and generalized velocities $dot(vb(q))$
- scalar Lagrange function $cal(L)$, is the difference between total kinetic energy and total potential energy
- Euler-Lagrange equation (of the second kind)
$
  dv(, t)(pdv(cal(L), dot(vb(q)))) - pdv(cal(L), vb(q)) = vb(tau)
$

kinetic energy of a system is defined as
$
  T = sum_(i=1)^(n_b) (1/2 m_i bl(dot(vb(r))_(S i), fa) dot bl(dot(vb(r))_(S i), fa) + 1/2 bl(vb(Omega)_(S i), fb) dot bl(vb(I)_(S i), fb)dot bl(vb(Omega)_(S i), fb))
$
velocities of each body is function of generalized velocities given by geometric Jacobian
$
  dot(vb(r))_(S i) = vb(J)_(P i) dot(vb(q)) quad vb(Omega)_(S i) = vb(J)_(R i) dot(vb(q))
$
then kinetic energy is expressed in generalized coordinates by
$
  T(vb(q),dot(vb(q))) = 1/2 dot(vb(q))^T (sum_(i=1)^n_b (vb(J)_(P i)^T m_i vb(J)_(P i) + vb(J)_(R i)^T vb(I)_(S i) vb(J)_(R i))) dot(vb(q)) = 1/2 dot(vb(q))^T vb(M)(vb(q)) dot(vb(q))
$

potential energy is from gravity and elastic energy
$
  V_g = - sum_(i=1)^n_b vb(r)_(S i) dot vb(F)_(g i) quad V_(E j) = 1/2 k_j (d_j (vb(q))-d_(j 0))^2
$

some constriants are defined by velocities are motion constriants, which are expressed as linear combinations of general velocities
$
  sum_(k=1)^n_q a_(k,j) (vb(q)) dot(q)_k + a_(0,j)(t)
$
for $n_(c,m)$ motion constriants, motion constriant Jacobian matrix is
$
  vb(J)_m = mat(a_(1,1), dots.c, a_(1,n_(c,m)); dots.v, dots.down, dots.v; a_(n_q,1), dots.c, a_(n_q,n_(c,m)))
$
some constriants are defined as function of configuration are configuration constriants, $f_j (vb(q)): RR^(n_q) -> RR$

constrianed Euler-Lagrange (CEL) equation (a.k.a Euler-Lagrange of the first kind) is given by:
$
  dv(, t)(pdv(cal(L), dot(vb(q)))) - pdv(cal(L), vb(q)) + vb(J)_m^T vb(lambda)_m + (pdv(vb(f), vb(q)))^T vb(lambda)_c = vb(tau)
$
where $vb(lambda)_m in RR^(n_(c,m))$ is vector of Lagrangian multipliers for motion constriants, $vb(lambda)_c in RR^(n_(c,c))$ is vector of Lagrangian multipliers for configuration constriants

== Projected Newton-Euler method

a combination of classical Newton-Euler equation for dynamic equilibrium in Cartesian coordinates and constriant compliant Lagrange formulation using generalized coordinates

change of linear and angular momentum can be rewritten using generalized coordinates
$
  vec(dot(vb(p))_S, dot(vb(N))_S) =vec(m vb(J)_(S i), vb(I)_(S i) vb(J)_(R i)) dot.double(vb(q)) + vec(m dot(vb(J))_(S i) dot(vb(q)), vb(I)_(S i) dot(vb(J))_(R i) dot(vb(q))+ vb(J)_(R i) dot(vb(q)) times vb(I)_(S i) vb(J)_(R i) dot(vb(q)))
$

virtual work principle can be rewritten using generalized coordinates
$
  0 = delta W = delta vb(q)^T sum_(i=1)^n_b vec(vb(J)_(P i), vb(J)_(R i))^T (vec(dot(vb(p))_(S i), dot(vb(N))_(S i)) - vec(vb(F)_("ext",i), vb(T)_("ext",i))) quad forall delta vb(q)
$

combining above two equations yields
$
  0 = sum_(i=1)^n_b [
    vec(vb(J)_(P i), vb(J)_(R i))^T vec(m vb(J)_(P i), vb(I)_(S i) vb(J)_(R i)) dot.double(vb(q)) +
    vec(vb(J)_(P i), vb(J)_(R i))^T vec(m dot(vb(J))_(P i) dot(vb(q)), vb(I)_(S i) dot(vb(J))_(R i) dot(vb(q))+ vb(J)_(R i) dot(vb(q)) times vb(I)_(S i) vb(J)_(R i) dot(vb(q)))
    - vec(vb(J)_(P i), vb(J)_(R i))^T vec(vb(F)_("ext",i), vb(T)_("ext",i))]
$
some terms are defined and computed as
$
  vb(M) &= sum_(i=1)^n_b vec(vb(J)_(P i), vb(J)_(R i))^T vec(m vb(J)_(S i), vb(I)_(S i) vb(J)_(R i)) = sum_(i=1)^n_b (bl(vb(J), fa)_(P i)^T dot m dot bl(vb(J), fa)_(P i) + bl(vb(J), fb)_(R i)^T dot bl(vb(I), fb)_(S i) dot bl(vb(J), fb)_(R i)) \
  vb(b) &= sum_(i=1)^n_b vec(vb(J)_(P i), vb(J)_(R i))^T vec(m dot(vb(J))_(P i) dot(vb(q)), vb(I)_(S i) dot(vb(J))_(R i) dot(vb(q))+ vb(J)_(R i) dot(vb(q)) times vb(I)_(S i) vb(J)_(R i) dot(vb(q))) \
  &= sum_(i=1)^n_b (bl(vb(J), fa)_(P i) dot m dot bl(dot(vb(J)), fa)_(P i) dot dot(vb(q)) + bl(vb(J), fb)_(R i)^T dot (bl(vb(I), fb)_(S i) dot bl(dot(vb(J)), fb)_(R i) dot dot(vb(q))+ bl(vb(Omega), fb)_(S i) times bl(vb(I), fb)_(S i) dot bl(vb(Omega), fb)_(S i))) \
  vb(g) &= sum_(i=1)^n_b - vb(J)_(S i)^T vb(F)_(g,i) = sum_(i=1)^n_b - bl(vb(J), fa)_(S i)^T dot bl(vb(F), fa)_(g,i)
$

there are $n_(f,"ext")$ external forces $vb(F)_j$ and $n_(m,"ext")$ external torques $vb(T)_k$ applied on system, based on virtual work, we have
$
  vec(delta vb(r)_(S 1), dots.v, delta vb(r)_(S n_(f,"ext")), vb(delta Phi)_1, dots.v, vb(delta Phi)_n_(m,"ext")) dot vec(vb(F)_1, dots.v, vb(F)_n_(f,"ext"), vb(T)_1, dots.v, vb(T)_n_(m,"ext")) = delta vb(q) dot vb(tau)_"ext" => \
  (delta vb(q))^T (sum_(j=1)^n_(f,"ext") vb(J)_(P,j)^T vb(F)_j + sum_(k=1)^n_(m,"ext") vb(J)_(R,k)^T vb(T)_k) = (delta vb(q))^T vb(tau)_"ext" => vb(tau)_"ext" = sum_(j=1)^n_(f,"ext") vb(J)_(P,j)^T vb(F)_j + sum_(k=1)^n_(m,"ext") vb(J)_(R,k)^T vb(T)_k
$

there are actuator forces $vb(F)_(a,k)$ / torques $vb(T)_(a,k)$ applied on joint k, which are imposed on two bodies equally and in opposite directions
$
  vb(tau)_(a,k) = (vb(J)_(P_k) - vb(J)_(P_(k-1)))^T vb(F)_(a,k) + (vb(J)_(R_k) - vb(J)_(R_(k-1)))^T vb(T)_(a,k)
$

total generalized forces is combination of joint actuators and body links:
$
  vb(tau) = sum_(k=1)^n_A vb(tau)_(a,k) + vb(tau)_"ext"
$

with contact forces, resulting equations of motion is
$
  vb(M) dot.double(vb(q)) + vb(b) + vb(g) = vb(tau) + vb(J)_c^T vb(F)_c
$

a multi-body dynamics formulation for fixed-based robots is
$
  vb(M)(vb(q)) dot.double(vb(q)) + vb(b)(vb(q),dot(vb(q))) + vb(g)(vb(q)) = vb(tau) + vb(J)_c^T (vb(q)) vb(F)_c
$
where $vb(M)(vb(q)) in RR^(n_q times n_q)$ is generalized mass matrix, $vb(q),dot(vb(q)),dot.double(vb(q)) in RR^(n_q)$ are generalized position, velocity and acceleration vectors, $vb(b)(vb(q),dot(vb(q))) in RR^(n_q)$ is Coriolis and centrifugal force terms, $vb(g)(vb(q)) in RR^(n_q)$ is gravitational terms, $vb(tau) in RR^(n_q)$ is external generalized forces (applied on bodies and joints), $vb(F)_c in RR^(n_c)$ is external Cartesian forces (from contacts), $vb(J)_c (vb(q)) in RR^(n_c times n_q)$ is geometric Jacobian corresponding to external forces

== Dynamics of floating base system

a multi-body dynamics formulation for floating-based robots is
$
  vb(M)(vb(q)) dot(vb(u)) + vb(b)(vb(q),vb(u)) + vb(g)(vb(q)) = vb(S)^T vb(tau) + vb(J)_"ext"^T vb(F)_"ext"
$
where $vb(q) in RR^(n_q)$ is generalized coordinates consists of actuated joint coordinates $vb(q)_j$ and unactuated base coordinates $vb(q)_b$, corresponding velocities are $vb(u)_j = dot(vb(q))_j in RR^(n_j)$ and $vb(u)_b in RR^6$. selection matrix selects joint velocity by
$
  vb(u)_j = vb(S) vb(u) = mat(vb(0)_(6 times 6), vb(I)_(6 times n_j)) vec(vb(u)_b, vb(u)_j)
$
to control unactuated base coordinates, external forces are necessary, they can come from contacts with environment (e.g. legged robots) or from aerodynamics (e.g. flying robots)

for legged robots, equation of motion is written as
$
  vb(M)(vb(q)) dot(vb(u)) + vb(b)(vb(q),vb(u)) + vb(g)(vb(q)) + vb(J)_c^T vb(F)_c = vb(S)^T vb(tau)
$<eq:3.56>
where $vb(F)_c$ is force that robot exerts on environment

there two different methods to model contact force:
- soft contact method models interaction by force element (i.e. spring-damper), contact force is function of location and velocity of point in contact.
$
  vb(F)_c = k_p (vb(r)_c - vb(r)_(c 0)) + k_d dot(vb(r))_c
$
- hard contact method models contact as a kinematic constriant, if a point is in contact, it's not allowed to move anymore
$
  cases(
    vb(r)_c = "const",
    dot(vb(r))_c = vb(J)_c vb(u) = vb(0),
    dot.double(vb(r))_c = vb(J)_c dot(vb(u)) + dot(vb(J))_c vb(u) = vb(0)
  ) =>
  vb(F)_c = (vb(J)_c vb(M)^(-1) vb(J)_c^T)^(-1)(vb(J)_c vb(M)^(-1)(vb(S)^T vb(tau)-vb(b)-vb(g))+dot(vb(J)_c) vb(u))
$<eq:3.61>
where equation of motion is #ref(<eq:3.56>)

substituting contact force #ref(<eq:3.61>) into equation of motion #ref(<eq:3.56>) results in
$
  vb(M) dot(vb(u)) + vb(b) + vb(g) + vb(J)_c^T (vb(J)_c vb(M)^(-1) vb(J)_c^T)^(-1)(vb(J)_c vb(M)^(-1)(vb(S)^T vb(tau)-vb(b)-vb(g))+dot(vb(J)_c) vb(u)) = vb(S)^T vb(tau) \
  => vb(N)_c^T (vb(M) dot(vb(u)) + vb(b) + vb(g)) = vb(N)_c^T vb(S)^T vb(tau) quad vb(N)_c^T = vb(I) - vb(J)_c^T (vb(J)_c vb(M)^(-1) vb(J)_c^T)^(-1)vb(J)_c vb(M)^(-1)
$<eq:3.64>
then dynamically consistent support null-space matrix/projector is defined as
$
  vb(N)_c = vb(I) - vb(M)^(-1) vb(J)_c^T (vb(J)_c vb(M)^(-1) vb(J)_c^T)^(-1)vb(J)_c
$

impact occurs when bodies collide with each other in a very short duration and high peak forces that results in energy dissipation and large acceleration

ignore all kinds of forces except contact impuse force, the equation of motion is integrated over impulse duration, results in
$
  vb(M) (vb(u)^+ - vb(u)^-) + vb(J)_c^T vb(cal(F))_c = vb(0)
$
for a perfect inelastic collision with a Newtonian collision law, all contact points instantaneously come to rest i.e. $dot(vb(r))_c^+ = vb(J)_c vb(u)^+ = vb(0)$, then impulsive force is give by
$
  vb(cal(F))_c = (vb(J)_c vb(M)^(-1) vb(J)_c^T)^(-1) vb(J)_c vb(u)^- = vb(Lambda)_c dot(vb(r))_c^-
$
where $vb(Lambda)_c$ is the inertia that is seen at support so called end-effector inertia, the post-impact generalized velocity is given by
$
  vb(u)^+ = vb(N)_c vb(u)^-
$
the pre-impact generalized velocity is projected onto support consistent manifold

the kinetic energy loss is give by
$
  Delta E_"kin" = -1/2 Delta vb(u)^T vb(M) Delta vb(u) = -1/2 Delta dot(vb(r))_c^T vb(Lambda) Delta dot(vb(r))_c = -1/2 dot(vb(r))_c^(- T) vb(Lambda)_c dot(vb(r))_c^-
$

== Joint-space dynamic control

for a torque controllable actuator, joint feedback control law is
$
  tau^star = k_p (q^star - q) + k_d (dot(q)^star - dot(q))
$
when applying this control law to a robot arm (without contact),
$
  M dot.double(q) + b(q,dot(q)) + g(q) = k_p (q^star - q) + k_d (dot(q)^star - dot(q))
$
there is a steady-state tracking error $g(q)$ if $dot(q)^star = 0, dot(q)->0, q->q^star + g(q)$

to compensate this steady-state offset, a common approach is
$
  tau^star = k_p (q^star - q) + k_d (dot(q)^star - dot(q)) + hat(g)(q)
$
where $hat(g)(q)$ is estimated gravity effects

inverse dynamics control is given by
$
  tau = hat(M) dot.double(q)^star + hat(b) + hat(g)
$
where $hat(M),hat(b),hat(g)$ are estimates. applying control law for a exact model results in
$
  dot.double(q) = dot.double(q)^star
$
a common approach to select desired acceleration is
$
  dot.double(q)^star = k_p (q^star - q) + k_d (dot(q)^star - dot(q))
$
which corresponds to a mass-spring-damper system with eigenfrequcy and damping value
$
  omega = sqrt(k_p) quad D = k_d / (2 sqrt(k_p))
$

== Task-space dynamics control

in most cases, we want move end-effector in task-space i.e. in world-fixed frame, the generalized acceleration is given by geometric Jacobian
$
  dot(vb(w))_e = vec(dot.double(vb(r)), dot(vb(omega)))_e = vb(J)_e dot.double(vb(q))+ dot(vb(J))_e dot(vb(q))
$

multi-task control is similar to kinematic multi-tasks control

end-effector dynamics is given by
$
  dot(vb(w))_e = vb(J)_e (vb(M)^(-1) (vb(tau)-vb(b)-vb(g))) + dot(vb(J))_e dot(vb(q))
$
relationship between joint torque and end-effector force is given by
$
  vb(tau) = vb(J)_e^T vb(F)_e
$
then task-space equations of motion is
$
  vb(Lambda)_e dot(vb(w))_e + underbrace(vb(Lambda)_e vb(J)_e vb(M)^(-1) vb(b) - vb(Lambda)_e dot(vb(J))_e dot(vb(q)), vb(mu)) + underbrace(vb(Lambda)_e vb(J)_e vb(M)^(-1) vb(g), vb(p)) = vb(F)_e quad vb(Lambda)_e = (vb(J)_e vb(M)^(-1) vb(J)_e^T)^(-1)
$
where the lhs consists of end-effector inertia, centrifugal/Coriolis, and gravitational terms

end-effector motion control is given by
$
  vb(tau)^star = hat(vb(J))_e^T (hat(vb(Lambda))_e dot(vb(w))_e^star + hat(vb(mu)) + hat(vb(p)))
$
where desired acceleration is
$
  dot(vb(w))_e^star = vb(k)_p vec(vb(r)_e^star - vb(r)_e, Delta vb(phi.alt)_e) + vb(k)_d (vb(omega)_e^star - vb(w)_e)
$
where $Delta vb(phi.alt)_e$ is end-effector rotation error

when robot should apply force in some directions and move in other directions, we need operational space control

== Inverse dynamics for floating-base system

given a desired acceleration, the #ref(<eq:3.64>) is inverted into
$
  vb(tau)^star = (vb(N)_c^T vb(S)^T)^+ vb(N)_c^T (vb(M) dot(vb(u))^star + vb(b) + vb(g))
$
depending on structure of $(vb(N)_c^T vb(S)^T)$, there is a null-space to modify torque
$
  vb(tau)^star = (vb(N)_c^T vb(S)^T)^+ vb(N)_c^T (vb(M) dot(vb(u))^star + vb(b) + vb(g)) + cal(N)(vb(N)_c^T vb(S)^T) vb(tau)^star_0
$

there are many approaches to solve problem of simultaneously controlling different operational-space objectives, which involve motion at selected locations (e.g. end-effector, CoG), desired contact forces, or joint torques

a method is to take operational space control as sequential least square optimization problem of linear objectives. problems with same priority are stacked in $vb(A)_i$ and $vb(b)_i$, $i=1$ is the highest priority, the hierarchical least square optimization of $n_T$ problems is to solve each task in a least square sense
$
  min_vb(x) norm(vb(A)_i vb(x) - vb(b)_i)_2
$
without influencing task of higher priority, methods includes iterative null-space projection and solving a sequence of constrianed quadratic programs using standard numerical solvers
