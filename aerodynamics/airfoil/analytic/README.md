# Analytic airfoil coefficients

This project contains implementations to fit analytic functions to airfoil data.
The optimized parameters can be used to perform efficient aerodynamic coefficient computations of your airfoil.
The airfoil data can not only depend on the angle of attack but also on the Mach number and on actuator states.

## Motivation

Sometimes it is more efficient or desirable to use analytic functions to compute aerodynamic coefficients of an airfoil
than to perform interpolations of (huge) maps.  
Moreover, your data might not cover the whole range of the input variables.
In this case interpolation and extrapolation might yield unreasonable results while the analytic
functions in this implementation contain a priori knowledge about the shape of the curves.  
Please refer to the documentation of each [content](#contents) for more information.

## Installation

- You must install [LADAC](../../../README.md)
- You need the MATLAB Optimization Toolbox.


## Contents

- [**Analytic airfoil: AoA from -5° to 15°**](analytic_05_15_alpha)  
With this project, lift, drag and pitching moment coefficients can be fitted for different Mach numbers and actuator states.
It can also be used for dynamic stall models.
- [**Analytic airfoil: AoA from -90° to 90°**](analytic_90_90_al)  
With this project, you can fit lift and drag coefficients for a high range of angles of attack.
However, this project is not yet ready to use it for dynamic stall models.
- [**Analytic airfoil: Beddoes-Leishman model (AoA from -5° to 25°)**](analytic_BL_alpha)  
This project implements the analytic functions published by Beddoes and Leishman which they used for dynamic stall models.
Note that the [Analytic airfoil: AoA from -5° to 15°](analytic_05_15_alpha) project has shown to give better fits for thick cambered airfoils.
- [**Analytic airfoil: simple model without stall**](analytic_simple)  
With this project, the lift, drag and pitching moment coefficient can be computed with a minimum number of parameters.
