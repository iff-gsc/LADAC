# Aerodynamics

The aerodynamics library is part of LADAC and contains functions and Simulink blocks to compute aerodynamics
of wings, airfoils, fuselages, ... with different methods.
Please refer to the documentation of each [content](#Contents) for more information.


## Contents

- [**Airfoil**](airfoil)  
The airfoil project contains models to fit airfoil coefficients with analytic functions 
and to compute unsteady airfoil aerodynamics with a continuous state-space model.
- [**Common functions**](common_functions)  
Common functions for aerodynamics models are located here.
- [**Fuselage**](fuselage)  
This is a very simple aerodynamics model for the fuselage that is based on
analytic aerodynamics coefficients functions depending on the angle of attack and the sideslip angle.
The functions parameters are estimated by test data available in literature.
- [**Rotorcraft aerodynamics**](rotorcraft_aerodynamics)  
The rotorcraft aerodynamics contains models to calculate the behavior of rotor disks based on the momentum theory.
It also contains a vortex ring state model that is based on real rotor test data.
Both steady and unsteady blade element momentum methods are implemented.
- [**Simple wing**](simple_wing)  
The simple wing project is 2-point aerodynamics model for wings that computes reasonable results even
for high angles of attack and sideslip angles, where potential flow methods fail.
However, for small aerodynamic angles this method is most likely less accurate as potential flow methods.
The validation of the simple wing project is currently underway.
- [**VLM Wing**](vlm_wing)  
The VLM wing allows the computation of wing aerodynamics with a nonlinear vortex lattice method (VLM)
that has no discretization in chord direction.
Moreover, it can be coupled with viscous airfoil data.
Both steady and unsteady computations are supported.
The wing project also contains an interface to structural dynamics models to consider an aeroelastic wing.
