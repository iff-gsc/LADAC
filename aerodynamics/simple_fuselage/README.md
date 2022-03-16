# Simple Aerodynamic Model for Fuselages

This project implements the aerodynamics of a fuselage at a reference point.
It is a semi-empirical model based on basic literature [1], [2].

Known issues:  
Currently, it is assumed that the reference point is the center of gravity. That should be changed in the future.


## Motivation 

For flight dynamic simulations, the fuselage often cannot be neglected when calculating aerodynamic forces and moments.
The fuselage mainly contributes an aerodynamic pitch and yaw moment.
However, it also happens that the fuselage generates a non-negligible lift and lateral force.
A fuselage model then influences the trim point and dynamic characteristics.
With this fuselage model, aerodynamic force and moment coefficients can be considered depending on the aerodynamic angles based on simple functions [2].


## Installation

- You need [LADAC](../../README.md)


## Example

Start the example by running `simpleFuselage_lib_example_init.m`:
  ```
  simpleFuselage_lib_example_init
  ```


## How to use

You can define your desired simple fuselage:
- Take a look at `simpleFuselageCreate`. You will see that you need to specify a parameters file.
Therefore, you should use the `simpleFuselage_params_default` file as a template.
- Make a copy of `simpleFuselage_params_default` and name it `simpleFuselage_params_myfuselage`.
- Adjust the parameters according to the documentation/comments.
- Now open the script `simpleFuselage_lib_example_init` and change the input variables of the function call `simpleFuselage_lib_example_init`.
Amend it to the name given to your parameter file.
- Run `simpleFuselage_lib_example_init` again and you will see the geometry of your fuselage as well as the lift distribution along the centerline.
- For more information, please study the scrip `simpleFuselage_lib_example_init`.
- All functions are documented. Take a look at them if you have questions.


## Literature

[1] Schlichting, H., & Truckenbrodt, E. (2001). Aerodynamik des Flugzeuges. Zweiter Band: Aerodynamik des Tragflügels (Teil II), des Rumpfes, der Flügel-Rumpf-Anordnung und der Leitwerke. 3. Auflage. Springer-Verlag Berlin Heidelberg.  
[2] Beyer, Y. (2017). Flight Control Design and Simulation of a Tandem Tilt Wing RPAS, Masterarbeit, Institute of Flight Guidance, TU Braunschweig.