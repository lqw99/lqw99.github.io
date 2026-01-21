#import "@preview/physica:0.9.7": *
#set math.equation(numbering: "(1)")
#let fa = $cal(A)$
#let fb = $cal(B)$
#let bl(v1, v2) = $attach(v1, bl: v2)$
= Optimization-Based Control for Dynamic Legged Robots


== Introduction

two main types of optimization-based control:
- predictive: consider an explicit system model to reason about the consequences of actions, iteratively devising and improving motion plans in response to the situation at hand.
- reactive: only consider actions for current instant


== Problem statement and overview

a high-level form of an optimal control problem (OCP) is written as
$
  min_(x,u,lambda) J(x,u,lambda) \
  s.t. quad M(q) dot(v) + C(q,v) v + tau_g (q) = S^T tau + J_c (q)^T lambda \
  "ContactConstraints"(x(t),lambda(t),u(t),"env") \
  "KinematicsConstraints"(x(t)) \
  "InputConstriants"(u(t),x(t)) \
  "TaskConstraints"(x(t),u(t),lambda(t))
$

the decision variables of OCP are trajectory of state $x = (q,v)$, control $u eq.delta tau$, and contact forces $lambda$ between robot and environment

there are several challenges in OCP, the nonsmoothness/stiffness of dynamic, nonconvexity, and dimensionality, summary as below

- contact models
contacts can be modeled as rigid or visco-elastic, visco-elastic models ensure continuous dynamics and smoothing techniques are used to make them differentiable, but the large stiffness value leads to stiff differential equations with numerical challenges for simulation and optimization; rigid contact model is a hybrid dynamic system, resulting OCP be tackled as a linear complementarity program (LCP) or as a mixed integer program (MIP), requires customized optimization techniques; when user fix the order in which contacts are made and broken, dynamics is time-switched (a special case of hybrid dynamics) and OCP is differentiable

- dynamic models

robot dynamics are high-dimensional and nonlinear, the resulting optimization problem is high-dimensional and nonconvex, several simplified models capture the most important part of robot dynamics with a reduced state size

- optimal control solution methods

direct methods, indirect methods, dynamic programming based methods. the optimization formulation needs to be approximated as a finite-size nonlinear program (NLP), the process is called transcription


== Contact

physical and frictional contact with environment leads to stiff/discontinuous equations of motion, how to model effects of contact and sequencing or scheduling of contact events are important
