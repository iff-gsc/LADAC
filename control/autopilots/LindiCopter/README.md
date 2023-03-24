# LindiCopter - LADAC INDI Copter Autopilot

LindiCopter is an autopilot for multicopters.
It is based on the control method incremental nonlinear dynamic inversion (INDI).
There is one inner control loop for the attitude and one outer control loop for the position.
In both control cascades INDI is used.
Currently, two flight modes are available: loiter and flight path following.
The flight path following flight mode uses splines based on defined waypoints.

## Installation

- You need [LADAC](../../README.md)

## How to use?

The autopilot Simulink block is called __LindiCopter Autopilot__ and is located in [lindiCopter_lib](lindiCopter_lib.slx).
The autopilot parameters can be computed automatically using the function [lindiCopterAutoCreate](lindiCopterAutoCreate.m) based on a quadcopter parameters struct (see [copter_params_default](../../../aircraft/multicopters/complete/quadcopter/copter_params_default.m)):
```
copter = copterLoadParams(copter_params_default');
lindiCopter_params = lindiCopterAutoCreate(copter);
```
For tuning have a look at the optional input arguments of the [lindiCopterAutoCreate](lindiCopterAutoCreate.m) function.

**Inputs:**

Input | Explanation
--- | ---
flightmode_number | 0: Loiter, 1: Guided, 2: Stabilized, 3: Altitude Hold, 4: AutoTune
cmd_roll_-1_1 | Roll stick command (-1 ... 1)
cmd_pitch_-1_1 | Pitch stick command (-1 ... 1)
cmd_yaw_-1_1 | Yaw stick command (-1 ... 1)
cmd_throttle_-1_1 | Throttle stick command (-1 ... 1)
is_cmd_in_NED | Define whether the commanded velocities are in FRD or NED frame (currently not supported)
waypoints | Waypoints (4xN array), first three rows is position in NED frame (in meters), fourth row is velocity in m/s, N is number of waypoints
num_waypoints | Number of waypoint (should match with input `waypoints`)
trigger_wp_update | This input triggers an internal update of the waypoints, the splines will be recomputed
M_bg | Rotation matrix (3x3) from geodetic frame g (NED) to body-fixed frame b (FRD)
omega_Kb | Angular velocity (3x1) of body w.r.t. earth represented in body-fixed frame b, in rad/s
s_Kg | Position (3x1) in geodetic frame g, in m
s_Kg_dt | Time-derivative of position s_Kg (3x1), m/s
s_Kg_dt2 | Second time-derivative of position s_Kg (31), in m/s/s
external_reset | Reset controller states (currently not supported)

**Outputs:**  
The output is a Simulink bus.
Have a look inside the block if the signal names are not clear.

**EKF and Flight Test:**  
LindiCopter does not contain an extended Kalman filter (EKF) for state estimation and can not be used direction for flight tests.
However, you can use the [ArduPilot custom controller interface](https://github.com/iff-gsc/LADAC/tree/main/utilities/interfaces_external_programs/ArduPilot_custom_controller).
Using the ArduPilot custom controller interface means, that the ArduPilot EKF is used.
With the ArduPilot custom controller interface you can perform software in the loop simulations as well as flight tests.

As a starting point there is an example in [LADAC-Examples](https://github.com/iff-gsc/LADAC-Examples).  
You can prepare the C++ Code Generation by running:  
[init_Arducopter_MinnieLindiCopter](Copter/Minnie/ArduPilot_implementation/init_Arducopter_MinnieLindiCopter.m).

**Flight Mode Info**  
- Loiter: Note that the Loiter flight mode currently runs in NED frame.
- Guided: Note that the Guided flight mode has currently a limit of 6 waypoints. Moreover, the autopilot currently may not enter the Guided flight mode in flight test (but it will work in Simulink and ArduCopter SITL). We are working on a bug fix.
- AutoTune: This mode is based on the work:  
Smeur, E. J., Chu, Q., & De Croon, G. C. (2016). Adaptive incremental nonlinear dynamic inversion for attitude control of micro air vehicles. Journal of Guidance, Control, and Dynamics, 39(3), 450-461.  
The control effectiveness will be adjusted online by random excitations of the copter. This mode should be run for approximately 30 seconds. There may be issues for the yaw control effectiveness. Note that all adjustments are gone after reboot. Therefore, the Simulink signals `autotune.factors_G1` and `autotune.factors_G2` can be logged to adjust the copter parameters before the next flight test.

**Flight Test Info**  
You should not use LindiCopter for take off and landing.
Use any ArduCopter flight mode for take off and landing.
You can switch into LindiCopter when the copter is in the air.

