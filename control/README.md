# Control

The control library is part of LADAC and contains functions and Simulink blocks related to flight control.
The purpose of each [content](Contents) is very different.
In the future, as the library grows, this will probably be restructured.


## Contents

- [**Angles**](angles)  
Contains functions to compute angle errors.
- [**Autopilots**](autopilots)  
Contains autopilots for specific vehicle types.
- [**Control allocation**](control_allocation)  
This project contains control allocation algorithms.
- [**Filters**](filters)  
This project contains filters and also is the basis for other implementations that use specialized filters
(e.g. reference models in the [NDI](NDI) project.
- [**INDI**](INDI)  
INDI should contain models related to incremental nonlinear dynamic inversion.
However, the only content is currently control allocation for INDI.
This will probably be restructured in the future.
- [**Joystick**](joystick)  
The joystick project contains a Simulink joystick model that works on both Windows and Linux.
There are calibration files for different controllers.
If no suitable calibration file is available, there is a calibration function to add new joysticks.
- [**Linear systems**](linear_systems)  
Linear systems contains function related to linear system like different state-space models and transfer function.
However, this is not restricted to control and will probably be restructured in the future.
- [**NDI**](NDI)  
NDI is about control design with Nonlinear Dynamic Inversion (NDI).
It contains function to compute feedback gains for NDI systems.
Moreover, it contains a kinematic inversion for all types of multicopters.
- [**Trajectory**](trajectory)  
Trajectory is about autonomous flight along given waypoints.
It contains function to compute smooth trajectorys from waypoints. 
Functions for determining the closest point of the trajectory to the current aircraft position.
And to determine the reference variables for attitude and acceleration as a function of the current airspeed in order to be able to follow the trajectory exactly.
