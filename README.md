# Implementation and validation of a real-time simulation of a quadrotor aerial vehicle on a Raspberry Pi


Source code repository for my [Master thesis](https://repozitorij.fsb.unizg.hr/islandora/object/fsb%3A6119) at [University of Zagreb, Faculty of Mechanical Engineering and Naval Architecture](https://www.fsb.unizg.hr/index.php?fsbonline).

## Numerical simulation of quadrotor aerial vehicle maneuver using new integration procedure

Implementation of kinematic reconstruction of attitutde for unnmaned aerial vechicle utilizing new integration procedure.  New integration method of unit quaternions performs 
incremental **integration on the tangential space of the rotational manifold SO(3)** with a direct reconstruction of the orientation on the manifold of unit quaternions SU(2).
After solving algebraic system for acceleration, problem is 'lifted' to tangent space (Lie algebra) via dexp function (see `dexp.c`) where any
vector-space ODE integration method can be used. After Runge-Kutta integration method is used, integration point is 'pulled back' from tangent space
to manifold via exp function (see `exp.c`).
