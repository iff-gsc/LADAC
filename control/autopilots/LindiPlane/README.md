# LindiPlane - LADAC INDI Plane Autopilot

LindiPlane is an autopilot for airplanes.
It is based on the control method incremental nonlinear dynamic inversion (INDI).
There is one inner control loop for the attitude and one outer control loop for the position.
In both control cascades INDI is used.
Currently, two flight modes are available: attitude control flight mode and waypoint navigation flight mode.

## Installation

- You need [LADAC](https://github.com/iff-gsc/LADAC#readme)

## How to use?

The autopilot Simulink block is called __LindiPlane Autopilot__ and is located in [lindiPlane_lib](lindiPlane_lib.slx).
The autopilot parameters can be computed automatically using the function [lindiPlaneAutoCreate](lindiPlaneAutoCreate.m) based on an airplane parameters struct (see [airplane_params_default](../../../aircraft/airplanes/complete/conventional_airplane_single_engine/airplane_params_default.m)):
```
airplane = conventionalAirplaneLoadParams(airplane_params_default');
[lindi,lindi_notune] = lindiCopterAutoCreate(airplane);
```
For tuning have a look at the optional input arguments of the [lindiPlaneAutoCreate](lindiPlaneAutoCreate.m) function.

**Inputs:**

Input | Explanation
--- | ---
flightmode_number | 0: Waypoint navigation, 1: Attitude control
cmd_roll_-1_1 | Roll stick command (-1 ... 1)
cmd_pitch_-1_1 | Pitch stick command (-1 ... 1)
cmd_yaw_-1_1 | Yaw stick command (-1 ... 1)
cmd_throttle_0_1 | Throttle stick command (0 ... 1)
waypoints | Waypoints (3xN array), rows are position in NED frame (in meters), N is number of waypoints
num_waypoints | Number of waypoint (should match with input `waypoints`)
M_bg | Rotation matrix (3x3) from geodetic frame g (NED) to body-fixed frame b (FRD)
omega_Kb | Angular velocity (3x1) of body w.r.t. earth represented in body-fixed frame b, in rad/s
s_g | Position (3x1) in geodetic frame g, in m
V_Kg | Time-derivative of position s_g (3x1), m/s
a_Kg | Second time-derivative of position s_g (3x1), in m/s/s
V_A | Airspeed, in m/s

**Outputs:**  
The output is a Simulink bus.
Have a look inside the block if the signal names are not clear.

**EKF and Flight Test:**  
LindiPlane does not contain an extended Kalman filter (EKF) for state estimation and can not be used directly for flight tests.
However, you can use the [ArduPilot custom controller interface](https://github.com/iff-gsc/LADAC/tree/main/utilities/interfaces_external_programs/ArduPilot_custom_controller).
Using the ArduPilot custom controller interface means, that the ArduPilot EKF is used.
With the ArduPilot custom controller interface you can perform software in the loop simulations as well as flight tests.

As a starting point there is an example in [LADAC-Examples](https://github.com/iff-gsc/LADAC-Examples).  
You can prepare the C++ Code Generation by running [init_Funray_LindiPlane_AP_codegen](https://github.com/iff-gsc/LADAC-Examples/blob/main/Plane/Funray/init_Funray_LindiPlane_AP_codegen.m).

**Flight Mode Info**  
- Waypoint navigation: Have a look at the [Waypoint navigation](https://github.com/iff-gsc/LADAC/blob/main/control/wpnav/README.md) project to see how the flight path is generated (waypoints are connected by straight lines and circle segments).
  When entering the mode, the airplane will try to reach the first waypoint.
  If the distance to the first waypoint is relatively large, an approach path to the first waypoint will be created internally.
  Since the approach does not yet cover all kinds of scenarios, you should make sure that the first waypoint is approximately in the direction of flight.
  If this flight mode is activated repeatedly, the last active waypoint is flown to as the first waypoint.
  After reaching the last waypoint, the mission starts all over again.
  If this mode is used with ArduPilot, it must be ensured that the mission upload was successful.
  The waypoints are initialized with the home position, so the aircraft will fly to the home position if the upload was not successful.
  Also, the takeoff waypoint should be deleted, otherwise it will be interpreted as the first waypoint.
  The thrust lever has direct access to the motors (no speed control).
- Attitude control: In this flight mode the sticks command a desired roll angle and pitch angle.
  The thrust lever has direct access to the motors (no speed control).

**Flight Test Info**  
You should not use LindiPlane for take off and landing.
Use any ArduPlane flight mode for take off and landing.
You can switch into LindiPlane when the airplane is in the air.

