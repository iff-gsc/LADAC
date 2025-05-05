# Flexible-Body Equations of Motion (simple)

This project implements simple flexible-body equations of motion according to Waszak & Schmidt [1].

## Motivation

In flight dynamics, aircraft are often modeled as rigid bodies.
Sometimes, however, it is important to consider some degrees of structural freedom for flexible aircraft.
This project contains a simple implementation of the equations of motion for slightly flexible aircraft by considering a selection of mode shapes according to Waszak & Schmidt [1].

## Installation

- You need [LADAC](../../README.md)

## Example

- Initialize the parameters of the Simulink model by running the script:  
  ``
  flexibleBody_lib_example_init;
  ``
- Run the simulink model `flexibleBody_lib_example`:  
  ``
  sim('flexibleBody_lib_example');
  ``

## How to use

Have a look at the initialization script `flexibleBody_lib_example_init` as well as the [structural dynamics project](../structure).
The Simulink example `flexibleBody_lib_example` uses the Simulink library block `flexible body equations of motion (simple)`, which is located in the Simulink library `flexibleBody_lib`.
Have a look at the mask of that block.
For the global motion of the flexible body, the inputs and outputs (bus) are the same as for the [rigid-body equations of motion](../rigid_body).
Note that the global motion always refers to the instantaneous center of gravity.
For the structural deformation, there are the following additional signals:

Signal name | Explanation
--- | ---
generalized_load_vector | Forces and moments vector in modal coordinates ((N+6)x1 vector for N mode shapes); can be obtained by `structure_red.modal.T'*q`, where `q` (6*Mx1 for M nodes) are the concentrated node forces and moments (the first 6 elements are the forces and moments at the first node, ...)
eta | Mode shape deflections ((N+6)x1 vector for N mode shapes); the first 6 elements are zero, the first mode shape is the 7th element
eta_dt | Time derivative of eta
eta_dt2 | Second time derivative of eta

## Literature
[1] Waszak, M. R. & Schmidt, D. K. (1988). Flight dynamics of aeroelastic vehicles. Journal of Aircraft, 25(6), 563-571.
