# Flight control

## Overview

The `control` area contains methods and reusable building blocks ranging from low-level mathematical control functions to complete autopilots.

The main levels are:

1. control methods and control-effectiveness models,
2. reusable controller modules and control allocation,
3. flight modes, guidance, trajectories, and waypoint navigation,
4. complete LindiCopter and LindiPlane autopilots.

## Control methods

### Incremental nonlinear dynamic inversion

[control/INDI](../control/INDI) contains functions and Simulink blocks for incremental nonlinear dynamic inversion (INDI), including control-effectiveness handling for propulsion or control-surface configurations and functions for multicopter and fixed-wing command transformations.

INDI components are building blocks. They are not identical to LindiCopter or LindiPlane, which combine multiple controllers, flight modes, and logic into complete autopilot models.

### Nonlinear dynamic inversion

[control/NDI](../control/NDI) contains NDI-related functions, examples, and controller-design utilities, including multicopter acceleration-to-attitude transformations and state-space helper functions.

### Control allocation

[control/control_allocation](../control/control_allocation) contains weighted least-squares and INDI-oriented control-allocation functions.
These are used to distribute desired control effects among available actuators while considering limits and configuration-dependent effectiveness.

## Reusable controller modules

[control/controller_modules](../control/controller_modules) contains reusable modules for:

- quaternion attitude control,
- reduced-attitude control,
- altitude control,
- position control.

Additional low-level support is available in [control/filters](../control/filters), [control/linear_systems](../control/linear_systems), angle functions, and command-processing utilities.

## Flight modes and guidance

[control/flight_modes](../control/flight_modes) contains composed modes such as:

- attitude control,
- altitude hold,
- loiter.

[control/trajectory](../control/trajectory) contains trajectory creation, interpolation, matching, section evaluation, and Frenet–Serret-related calculations.

[control/wpnav](../control/wpnav) provides waypoint-navigation functions based on lines and circular segments.

The current source-tree names may evolve, but these modules remain conceptually separate from the complete autopilots.

## LindiCopter

[LindiCopter](../control/autopilots/LindiCopter) is a configurable multicopter autopilot based on INDI.

The current implementation includes inner attitude control, outer position-related control, flight-mode logic, waypoint/trajectory functions, and control allocation.
Its parameter structure can be generated from a quadcopter aircraft-model structure:

```matlab
copter = copterLoadParams('copter_params_default');
lindi = lindiCopterAutoCreate(copter);
```

Optional inputs to `lindiCopterAutoCreate` are used for tuning and configuration.

The current published implementation primarily targets quadcopters.
Motor positions and rotation directions are parameterized, providing a basis for future generalization to arbitrary rotor counts and layouts, but other configurations should not be considered supported without adaptation and testing.

LindiCopter does not include its own state estimator.
For ArduPilot SITL and flight testing, it can use the state estimate and infrastructure provided by the custom ArduPilot controller interface.

Consult the local [README](../control/autopilots/LindiCopter/README.md) for current flight modes, interfaces, limitations, and flight-test notes.

## LindiPlane

[LindiPlane](../control/autopilots/LindiPlane] is a configurable fixed-wing autopilot based on INDI.

It combines guidance, attitude and acceleration control, actuator-effectiveness modeling, and flight-mode logic.
Its parameter structure can be derived from a conventional-aircraft model structure:

```matlab
airplane = conventionalAirplaneLoadParams( 'airplane_params_default' );
lindi = lindiPlaneAutoCreate(airplane);
```

The current implementation focuses on a conventional configuration with main wing, horizontal tail, and vertical tail.
Adapting it to configurations such as flying wings or canards may require changes beyond parameter tuning.

LindiPlane does not include its own state estimator.
The custom ArduPilot controller interface can provide the ArduPilot EKF state estimate for SITL and flight testing.

Consult the local [README](../control/autopilots/LindiPlane/README.md) for current flight modes, interfaces, limitations, and flight-test notes.

## Controller-development workflow

1. Define and validate the aircraft model.
2. Generate or define control-effectiveness parameters.
3. Configure controller modules and filters.
4. Test each control loop independently where possible.
5. Test the complete controller in nonlinear simulation.
6. Include sensor noise, delay, actuator dynamics, saturations, disturbances, and parameter uncertainty.
7. Test mode transitions and reset behavior.
8. Use ArduPilot SITL before flight testing.
9. Retain a standard flight mode and tested failsafe behavior for recovery.

## Generated controller parameters

Where possible, aircraft properties should be defined once in the flight-dynamics parameter structure and transformed into controller parameters by an automatic creation function.
Automatically generated parameters do not eliminate the need for:

- controller tuning,
- sign and frame verification,
- filtering choices,
- validation of control-effectiveness assumptions,
- configuration-specific safety testing.

## Controller deployment

A custom controller can use LADAC functions without using LindiCopter or LindiPlane.

For deployment through the ArduPilot interface, the controller must be compatible with Simulink code generation and connected to the prescribed interface blocks.
See [ArduPilot integration](ArduPilot_Integration.md).
