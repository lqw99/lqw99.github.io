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
  vec(x_1, dots.v, x_N) = mat() z + vec() x_0
$
