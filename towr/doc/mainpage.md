\mainpage Test


[TOC]


Control Toolbox {#ct}
====================


This is the ADRL Control Toolbox ('CT'), an open-source C++ library for efficient modelling, control, 
estimation, trajectory optimization and model predictive control.
The CT is applicable to a broad class of dynamic systems, but features additional modelling tools specially designed for robotics.
This page outlines its general concept, its major building blocks and highlights selected application examples.

The library contains several tools to design and evaluate controllers, model dynamical systems and solve optimal control problems.
The CT was designed with the following features in mind:

 - **Systems and dynamics**: 
  - intuitive modelling of systems governed by ordinary differential- or difference equations.

 - **Trajectory optimization, optimal control and (nonlinear) model predictive control**:
    - intuitive modelling of cost functions and constraints
    - common interfaces for optimal control solvers and nonlinear model predictive control
    - currently supported algorithms:
      - Single Shooting
      - iLQR \cite todorov2005ilqg / SLQ \cite slq:2005 
    - Gauss-Newton-Multiple-Shooting (GNMS) \cite giftthaler2017family
    - Classical Direct Multiple Shooting (DMS) \cite bock1984direct
  - standardized interfaces for the solvers
    - IPOPT (first and second order)
      - SNOPT
      - HPIPM
      - custom Riccati-solver
 
Robot Application Examples {#examples}
==========================

The Control Toolbox has been used for Hardware and Simulation control tasks on flying, walking and ground robots.
