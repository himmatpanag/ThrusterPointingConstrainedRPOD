# Thruster Pointing Constrained Optimal Control for Satellite Servicing using Indirect Optimization
Code accompanying journal paper [1] 

Core functions: (also explained in 'ExampleCode.m') 

1. SolvePointingConstrainedControlProblem.m - Core function which implements the thruster pointing constrained control problem and solves is using a single shooting method. The constraint is illustrated in the figure below.

<p align="center" width="100%">
    <img width="50%" src="diagramConstraint.png"> 
</p>

2. ConstrainedApproachTestCondition.m - Contains the initial conditions for various transfer types.

3. SweepSolutions.m - Useful to sweep the smoothing paramete rho or target radius via continuation methods.

**References**

[1] H. Panag, R. Woollands; Thruster Pointing Constrained Optimal Control for Satellite Servicing using Indirect Optimization, Journal of Spacecraft and Rockets. 
