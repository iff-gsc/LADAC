# Discretized Aerodynamic Model for Fuselages

This program discretely resolves the aerodynamics of the fuselage along the centerline.
This is especially important for the simulation of aeroelastic models;
the coupling with a structural dynamics model is supported.
This aerodynamic model is based on basic equations from [1].


## Motivation 

For flight dynamic simulations, the fuselage often cannot be neglected when calculating aerodynamic forces and moments.
The fuselage mainly contributes an aerodynamic pitch and yaw moment.
However, it also happens that the fuselage generates a non-negligible lift and lateral force.
A fuselage model then influences the trim point and dynamic characteristics.
This fuselage model is discretely resolved along the centerline and offers the possibility to be coupled with a structural model.
Furthermore, this discrete model offers the possibility to consider simple aerodynamic interactions between fuselage and wing.


## Installation

- You need [LADAC](../../README.md)


## Examples

### General

Start the example by running `fuselage_example.m`:
  ```
  fuselage_example
  ```
Two figures should appear: lift distribution along the centerline and geometric visualization of the fuselage.

### Simulink

Start the Simulink example by running:
  ```
  fuselage_lib_example_init
  ```
Take a look at the Simulink model.
For information about the inputs and outputs, take a look at the Matlab function documentation ([see How to use](#How)).


## How to use

You can define your desired fuselage:
- Take a look at `fuselageCreate`. You will see that you need to specify a parameters file.
Therefore, you should use the `fuselage_params_default` file as a template.
- Make a copy of `fuselage_params_default` and name it `fuselage_params_myfuselage`.
- Adjust the parameters according to the documentation/comments.
- Now open the script `fuselage_example` and change the input variables of the function call `fuselageCreate`.
Amend it to the name given to your parameter file.
- Run `fuselage_example` again and you will see the geometry of your fuselage as well as the lift distribution along the centerline.
- For more information, please study the scrip `fuselage_example`.
- All functions are documented. Take a look at them if you have questions.


## Literature

[1] Schlichting, H., & Truckenbrodt, E. (2001). Aerodynamik des Flugzeuges. Zweiter Band: Aerodynamik des Tragflügels (Teil II), des Rumpfes, der Flügel-Rumpf-Anordnung und der Leitwerke. 3. Auflage. Springer-Verlag Berlin Heidelberg.
