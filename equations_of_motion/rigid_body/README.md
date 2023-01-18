# Rigid Body Equations of Motion

This project implements equations of motion for rigid bodies.

## Motivation

Often aircraft can be assumed to be rigid and are consequently modeled with rigid body equations of motion.
The Simulink blocks in `rigidBody_lib` are based on Matlab functions which are named `rigidBodyKinetics` and `rigidBodyKinematicsQuat`.


## Installation

- You need [LADAC](../../README.md)


## Test

Run the test function
```
rt = table(runtests('rigidBodyUnitTest'))
```

## How to use

Have a look at the Simulink blocks and at the Matlab functions.

Here are the naming conventions of the variables and Simulink signal names:

**Indices Naming Convention:**

Indices are added to the names after an underscore `_`.

Index | Explanation
--- | ---
b | Body-fixed frame with forward-right-down convention, with the origin at the center of gravity
g | Geodetic frame with north-east-down convention, with the origin at the center of gravity
K | Motion of the body with respect to the earth


**Signal/Variable Names:**

Signal name | Explanation
--- | ---
R_b | Force vector represented in body-fixed frame (3x1 array), in N
Q_b | Moment vector with respect to the center of gravity represented in body-fixed frame (3x1 array), in Nm
I_b | Inertia matrix represented in body-fixed frame (3x3 array), in kg.m^2
m | Mass of the body (scalar), in kg
g | Gravitational acceleration (scalar), in m/s^2
V_Kg | Velocity of the center of gravity relative to the earth represented in geodetic frame (3x1 array), in m/s
V_Kb | Velocity of the center of gravity relative to the earth represented in body-fixed frame (3x1 array), in m/s
V_Kb_dt | Time-derivative of V_Kb (3x1 array), in m/s^2 
s_g | Position with respect to the initial position represented in geodetic frame (3x1 array), in m
s_g_dt | Time-derivative of s_Kg, in m/s
omega_Kb | Angular velocity of the rigid body relative to the earth represented in body-fixed frame (3x1 array), in rad/s
omega_Kb_dt | Time-derivative of omega_Kb (3x1 array), in rad/s^2
q_bg | Quaternion from geodetic frame (g) to body-fixed frame (b) (3x1) 
q_bg_dt | Time-derivative of q_bg
EulerAngles | Euler angles (roll, pitch, yaw) (3x1 array), in rad
M_bg | Rotation matrix from geodetic frame (g) to body-fixed frame (b) (3x3 array)
