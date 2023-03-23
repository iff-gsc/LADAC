# LindiCopter - LADAC INDI Copter Autopilot

LindiCopter is an autopilot for multicopters.
It is based on the control method incremental nonlinear dynamic inversion (INDI).
There is one inner control loop for the attitude and one outer control loop for the position.
In both control cascades INDI is used.
Currently, two flight modes are available: loiter and flight path following.
The flight path following flight mode uses splines based on defined waypoints.

## Installation

- You need [LADAC](../../../#readme)

## How to use?

The autopilot Simulink block is called __LindiCopter Autopilot__ and is located in [`lindiCopter_lib`](lindiCopter_lib.slx).
The autopilot parameters can be computed automatically using the function [`lindiCopterAutoCreate`](lindiCopterAutoCreate.m) based on a quadcopter parameters struct (see [copter_params_default](../../../aircraft/multicopters/complete/quadcopter/copter_params_default.m)):
```
copter = copterLoadParams(copter_params_default');
lindiCopter_params = lindiCopterAutoCreate(copter);
```

**Inputs:**

Input | Explanation
--- | ---
flightmode_number | 0: loiter, 1: guided
cmd_lateral_velocity | commanded lateral velocity (-1 ... 1)
cmd_forward_velocity | commanded forward_velocity (-1 ... 1)
cmd_yaw_rate | commanded yaw rate (-1 ... 1)
cmd_throttle | commanded throttle (-1 ... 1)
is_cmd_in_NED | define whether the commanded velocities are in FRD or NED frame (currently not supported)
waypoints | waypoints (4xN array), first three rows is position in NED frame (in meters), fourth row is velocity in m/s, N is number of waypoints
trigger_wp_update | this input triggers an internal update of the waypoints, the splines will be recomputed
M_bg | rotation matrix (3x3) from geodetic frame g (NED) to body-fixed frame b (FRD)
omega_Kb | angular velocity (3x1) of body w.r.t. earth represented in body-fixed frame b, in rad/s
s_Kg | position (3x1) in geodetic frame g, in m
s_Kg_dt | time-derivative of position s_Kg (3x1), m/s
s_Kg_dt2 | second time-derivative of position s_Kg (31), in m/s/s
external_reset | reset controller states (currently only desired position will be resetted

**Outputs:**  
The output is a Simulink bus.
Have a look inside the block if the signal names are not clear.

**EKF and flight test:**  
LindiCopter does not contain an extended Kalman filter (EKF) for state estimation and can not be used direction for flight tests.
However, you can use the [ArduPilot custom controller interface](../../../utilities/interfaces_external_programs/ArduPilot_custom_controller#readme).
Using the ArduPilot custom controller interface means, that the ArduPilot EKF is used.
With the ArduPilot custom controller interface you can perform software in the loop simulations as well as flight tests.

