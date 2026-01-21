#import "@preview/physica:0.9.7": *


参考资料

[1] #link("http://cse.lab.imtlucca.it/~bemporad/mpc_course.html")[Model Predictive Control by Alberto Bemporad]

= Basic Concepts

MPC problem is to find best control sequence over a future horizon of N steps, and only apply optimal control of first time step

constrained optimization
$
  min_x quad & f(x) quad x in RR^n, f: RR^n -> RR \
   s.t. quad & g(x) <= 0 quad g: RR^n -> RR^m
$

convex set:
$
  lambda x_1 + (1-lambda) x_2 in S\
  forall x_1,x_2 in S subset.eq RR^n, lambda in [0,1]
$
convex function:
$
  f(lambda x_1 + (1-lambda) x_2) <= lambda f(x_1) + (1-lambda) f(x_2) \
  forall x_1,x_2 in S, lambda in [0,1]
$

convex optimization problem:
$
  min_x quad & f(x) quad f: S -> RR "is a convex function" \
   s.t. quad & x in S quad S "is a convex set"
$
where $S$ is often defined by linear equality constraints $A x = b$ and convex inequality constraints $g(x) <= 0, g: RR^n -> RR^m$, every local solution is also a global one.

some kinds of convex optimization problems:

- linear programming (LP) problem
  $
    min_x quad & c^T x \
     s.t. quad & A x <= b quad E x = f, quad x in RR^n
  $

- quadratic programming (QP) problem
  $
    min_x quad & 1/2 x^T Q x + c^T x \
     s.t. quad & A x <= b quad E x = f, quad x in RR^n
  $
where $Q$ is a positive semidefinite matrix

- mixed-integer programming (MIP) problem
  - mixed-integer linear programming (MILP)
  $
    min_x quad & c^T x \
     s.t. quad & A x <= b quad E x = f, quad x=vec(x_c, x_b) \
          quad & x_c in RR^(n_c), x_b in {0,1}^(n_b)
  $
  - mixed-integer quadratic programming (MIQP)
  $
    min_x quad & 1/2 x^T Q x + c^T x \
     s.t. quad & A x <= b quad E x = f, quad x=vec(x_c, x_b) \
          quad & x_c in RR^(n_c), x_b in {0,1}^(n_b)
  $

= Linear MPC

== unconstrained linear MPC

linear prediction model:
$
  x_(k+1) & = A x_k + B u_k quad x in RR^n, u in RR^m \
      y_k & = C x_k quad y in RR^p
$
where $x_k = A^k x_0 + sum_(j=0)^(k-1) A^j B u_(k-1-j)$ is a predictive state using $x_0 = x(t)$.

performance index is function of state of time $t$ and a sequence of control
$
  J(z,x_0) = x_N^T P x_N + sum_(k=0)^(N-1) (x_k^T Q x_k + u_k^T R u_k) quad z = vec(u_0, dots.v, u_(N-1))
$

goal is to find the optimal sequence $z^star$ that minimizes $J(z,x_0)$

performance index can be rewritten as
$
  J(z,x_0) = x_0^T Q x_0 + vec(x_1, dots.v, x_N)^T underbrace(
    mat(
      Q, dots.c, 0, 0;
      dots.v, dots.down, dots.v, dots.v;
      0, dots.c, Q, 0;
      0, dots.c, 0, P
    ), macron(Q)
  ) vec(x_1, dots.v, x_N)
  + z^T underbrace(
    mat(
      R, dots.c, 0;
      dots.v, dots.down, dots.v;
      0, dots.c, R
    ), macron(R)
  ) z \
  vec(x_1, x_2, dots.v, x_N) = mat(
    B, 0, dots.c, 0;
    A B, B, dots.c, 0;
    dots.v, dots.v, dots.down, dots.v;
    A^(N-1) B, A^(N-2) B, dots.c, B
  ) z + vec(A, A^2, dots.v, A^N) x_0 = macron(S) z + macron(T) x_0 \
  => J(z,x_0) = (macron(S) z + macron(T) x_0)^T macron(Q) (macron(S) z + macron(T) x_0) + z^T macron(R) z + x_0^T Q x_0 \
  = 1/2 z^T 2 (macron(R) + macron(S)^T macron(Q) macron(S)) z + x_0^T 2 macron(T)^T macron(Q) macron(S) z + 1/2 x_0^T 2 (Q + macron(T)^T macron(Q) macron(T)) x_0 = 1/2 z^T H z + x_0^T F^T z + 1/2 x_0^T Y x_0
$

the condensed form of MPC is
$
  J(z,x_0) = 1/2 z^T H z + x_0^T F^T z + 1/2 x_0^T Y x_0 quad z = vec(u_0, dots.v, u_(N-1))
$
the optimum is given by zeroing gradient
$
  grad_z J = H z + F x_0 = 0 => z^star = - H^(-1) F x_0
$
there are other ways:
- find $z^star$ via dynamic programming (Riccati iterations)
- keep $x_1,dots.c,x_N$ also as optimization variables and equality constraints $x_(k+1) = A_k x_k + B_k u_k$, which leads to a very sparse non-condensed form

at each time step $t$, the initial state is $x_0 = x(t)$, find optimal control trajectory $z^star$, the control input at time $t$ is $u(t) = u_0$, then the control law is
$
  u(t) = mat(I, 0, dots.c, 0) z^star = - mat(I, 0, dots.c, 0) H^(-1) F x(t) = K x(t)
$
so unconstrained linear MPC = linear state-feedback

== constrained linear MPC

for linear prediction model, there are constraints to enforce
$
  u_min <= u(t) <= u_max in RR^m \
  y_min <= y(t) <= y_max in RR^p
$
input constraints are rewritten as
$
  cases(&u_k<= u_max, -&u_k <= -u_min) =>
  mat(
    1, dots.c, 0;
    dots.v, dots.down, dots.v;
    0, dots.c, 1;
    -1, dots.c, 0;
    dots.v, dots.down, dots.v;
    0, dots.c, -1
  )
  vec(u_0, dots.v, u_(N-1)) <= vec(u_max, dots.v, u_max, -u_min, dots.v, -u_min) \
  => G_1 z <= W_1
$
output constraints are rewritten as
$
  y = mat(C, , ; , dots.down; , , C) vec(x_1, dots.v, x_N) = macron(C) macron(S) z + macron(C) macron(T) x_0 \
  mat(macron(C) macron(S); - macron(C) macron(S)) z <= vec(y_max, dots.v, y_max, -y_min, dots.v, -y_min) - mat(macron(C) macron(T); -macron(C) macron(T)) x_0 => G_2 z <= W_2 + S_2 x_0
$

condensed form of constrained optimal control problem with quadratic performance index is given by (matrices $H,F,Y$ are defined as above)
$
  min_z quad & J(z,x_0) = 1/2 z^T H z + x_0^T F^T z + 1/2 x_0^T Y x_0 \
   s.t. quad & G z <= W + S x_0 quad G = mat(G_1; G_2) quad W = mat(W_1; W_2) quad S = mat(0; S_2)
$

linear MPC with tracking

tracking makes output $y(t)$ track a reference signal $r(t)$, parameterize the problem using input increments $Delta u_k = u_k - u_(k-1)$, the system has a new state $x_(u,k) = u_(k-1)$
$
  vec(x_(k+1), x_(u,k+1)) = mat(A, B; 0, I) vec(x_k, x_(u,k)) + mat(B; I) Delta u_k \
  y_k = mat(C, 0) vec(x_k, x_(u,k))
$
corresponding optimal control problem is
$
  min_z quad & sum_(k=0)^(N-1) (norm(W^y (y_(k+1)-r(t)))_2^2 + norm(W^(Delta u) Delta u_k)_2^2)
               quad Delta u_k eq.delta u_k - u_(k-1) \
   s.t. quad & u_min <= u_k <= u_max \
        quad & y_min<= y_k <= y_max \
        quad & Delta u_min <= Delta u_k <= Delta u_max
$
