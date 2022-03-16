# Airfoil

The airfoil library is part of the aerodynamics library of LADAC and contains functions and Simulink blocks to compute airfoil
coefficients.
Please refer to the documentation of each [content](#contents) for more information.


## Contents

- [**Analytic**](analytic)  
The analytic project contains models to fit airfoil coefficients with analytic functions.
- [**Dynamic stall**](dynamic_stall)  
The dynamic stall project contains a reduced order dynamic stall model which allows the computation of unsteady aerodynamic coefficients beyond stall.
Therefore, it uses the [analytic](analytic) project and it extends the [unsteady](unsteady) project about additional states.
- [**Flap**](flap)  
The flap project allows the computation of steady and unsteady aerodynamic coefficients due to flap motion.
- [**Section database**](section_database)  
The section database contains some existing airfoil maps in specifically defined format.
Moreover, there is a function to read aerodynamic coefficients from XFOIL files.
- [**Unsteady**](unsteady)  
The unsteady project computes unsteady aerodynamic coefficients with a reduced order model.
This project is only applicable below stall.
For dynamic stall, please take a look at the [dynamic stall](dynamic_stall) project.
However, the unsteady project runs faster than the dynamic stall project.