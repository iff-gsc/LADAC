# Section database

The section database is an attempt to simplify and unify the computation of data-based airfoil coefficients.
Therefore, there are currently two main conventions.
1. Either use a struct and the lift, drag and pitching moment coefficients are only depending on the angle of attack (see [1. Load data from XFOIL files](#1-load-data-from-xfoil-files)).
2. Create and use a huge map where the lift, drag and pitching moment coefficients are functions of the angle of attack, the Reynolds number, the Mach number and actuator settings (see [2. Convention of ...](#2-convention)).  

Please note that the current state of this project is not really satisfactory and should be improved in the future.

## 1. Load data from XFOIL files

Take a look at the function `airfoilLoadXfoilData` and try the example:
```
NACA_63_412 = airfoilLoadXfoilData( 'xf-n63412-il-1000000' );
plot( NACA_63_412.alpha, NACA_63_412.c_L, 'x' )
```

## 2. Convention of How to Define Airfoil Aerodynamics Maps in .mat files.

This is a short explanation of how to work with airfoil aerodynamics maps stored in .mat files
or how to add new .mat files.

### Definition of the struct

The .mat files contain a struct that is defined as follows:

- grid (1x1 struct)
  - val (1x1 struct)
    - x1 (1xl array)
	- x2 (1xm array)
	- x3 (1xn array)
	- ...
  - name(1x1 struct)
    - x1 (char for identification)
	- x2 (char for identification)
	- x3 (char for identification)
	- ...
- data (1x1 struct)
  - c_L (lxmxn... array)
  - c_D (lxmxn... array)
  - c_m (lxmxn... array)
  
  
### Explanation of the variables inside the struct

There must be 3 maps in the struct: 
The lift coefficient, the drag coefficient and the pitching moment coefficient.

Variable | Explanation
--- | ---
c_L | lift coefficient
c_D | drag coefficient
c_m | pitching moment coefficient with respect to c/4

At the moment the maps can depend on up to 4 variables.
These variables are defined by the char for identification.
The order can be chosen freely.

Available chars for identification | Explanation
--- | ---
'alpha' | angle of attack
'Mach' | Mach number
'Reynolds' | Reynolds number
'actuator_1' | State of a 1st actuator
'actuator_2' | State of a 2nd actuator


### How to use?

The maps should be loaded by using the function
```
airfoil_map_struct = wingAirfoilMapLoadSection(names_of_mat_files);
```
Example: `airfoil_map = wingAirfoilMapLoadSection('F15_bl');`

If you want to use multiple maps in Simulink, thinks become very difficult due to code generation.
You should convert the maps (array of structs) with the function
```
airfoil_map_scell = wingAirfoilMapSetSim(airfoil_map_struct);
```
Example: `airfoil_map_scell = wingAirfoilMapSetSim(wingAirfoilMapLoadSection('F15_bl'));`

You now can use the function
```
coeff = wingAirfoilMapInterpCoeff( airfoil_map_scell, segment_type, type, alpha, Ma, Re, act1, act2 )
```
to get the aerodynamic coefficient (specify by `type`) for a specified airfoil (defined by `segment_type`).
If some variables, e.g. the Reynolds number, are not specified in the airfoil map, arbitrary values can be passed to the function.
The aerodynamic derivatives can be computed with `wingAirfoilMapDerivCoeff`.
