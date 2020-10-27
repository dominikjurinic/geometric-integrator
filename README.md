# Implementation and validation of a real-time simulation of a quadrotor aerial vehicle on a Raspberry Pi


Source code repository for my [Master thesis](https://repozitorij.fsb.unizg.hr/islandora/object/fsb%3A6119) at [University of Zagreb, Faculty of Mechanical Engineering and Naval Architecture](https://www.fsb.unizg.hr/index.php?fsbonline).

## Numerical simulation of quadrotor aerial vehicle maneuver using new integration procedure

Implementation of kinematic reconstruction of attitutde for unnmaned aerial vechicle utilizing new integration procedure.  New integration method of unit quaternions performs 
incremental **integration on the tangential space of the rotational manifold SO(3)** with a direct reconstruction of the orientation on the manifold of unit quaternions SU(2).
After solving algebraic system for acceleration, problem is 'lifted' to tangent space (Lie algebra) via dexp function (see `dexp.c`) where any
vector-space ODE integration method can be used. After Runge-Kutta integration method is used, integration point is 'pulled back' from tangent space
to manifold via exp function (see `exp.c`).

Relevant papers:
- [Lie-group integration method for constrained multibody systems in state space](https://link.springer.com/article/10.1007/s11044-014-9439-2?shared-article-renderer)

- [Singularity-free time integration of rotational quaternions using non-redundant ordinary differential equations](https://link.springer.com/article/10.1007/s11044-016-9518-7)

- [Aircraft attitude reconstruction via novel quaternion-integration procedure](https://www.sciencedirect.com/science/article/abs/pii/S1270963819304742)

- [Geometric integrators for numerical simulation of flight vehicle dynamics](https://repozitorij.fsb.unizg.hr/islandora/object/fsb%3A5907/datastream/PDF/view)

## Results Summary
Existing Matlab implementation was not satisfactory, as real time execution on Raspbbery Pi computer was not possible.In simulation, unnmaned aerial vechicle performs maneuver which lasts one second, so the execution time must be less than one second taking into account the safety margain.  The goal was, therefore, to implement and test metioned integration procedure first in Python then in C. The table below contains results of implementations for different time integration steps. 
Time step(s) | 0.01 | 0.001 | 0.0001
------------ | -------------  | -----------|------------|
Python| 0.33 | 1.21 |9.78   
C     |   -  |  0.005  |   0.22    | 


Results showed that it is possible to achieve real time execution for both implementations, but not for every timestep. Due to the anatomy of the new algorithm, it is possible to achieve the same level of accuracy, regardless of the time step, which is not a characteristic of classical integration methods in which the integration step correlates with accuracy

## Project status
No longer developed. 
- Goal was to test mentioned implementation on Raspberry Pi computer. This master thesis was part of a research project that aims to build a UAV that would serve as a platform for future research.

