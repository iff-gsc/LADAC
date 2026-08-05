# ArduPilot integration

LADAC provides two different interfaces related to ArduPilot:

1. an interface between a LADAC aircraft-dynamics model and ArduPilot SITL,
2. a toolchain for running generated MATLAB/Simulink controller code in a custom ArduPilot flight mode.

Do not confuse these workflows.

## 1. ArduPilot SITL with LADAC flight dynamics

The interface in [interfaces/ArduPilot_SITL](../interfaces/ArduPilot_SITL) exchanges simulation data between ArduPilot SITL and a MATLAB/Simulink aircraft model.

In this configuration:

- aircraft dynamics run in MATLAB/Simulink,
- the normal ArduPilot estimator and controller run in ArduPilot,
- ArduPilot sends actuator commands to the simulation,
- the simulation returns sensor or vehicle-state information through the interface.

This workflow is useful for testing ArduPilot with a custom LADAC vehicle model.

Consult the local README and [LADAC-Examples](https://github.com/iff-gsc/LADAC-Examples) for current setup instructions and tested configurations.

## 2. Generated MATLAB/Simulink controller in ArduPilot

The interface in [interfaces/ArduPilot_custom_controller](../interfaces/ArduPilot_custom_controller) allows a controller developed in MATLAB/Simulink to run inside ArduCopter or ArduPlane using Matlab Code Generation.

The ArduPilot fork adds a custom flight mode and controller interface while retaining the normal ArduPilot software and standard flight modes.

The custom flight mode:

- provides controller inputs from ArduPilot state estimation, navigation, pilot commands, and system data,
- assigns these values to the generated controller interface,
- calls the generated controller `step()` function,
- forwards controller outputs to motors or control surfaces,
- logs selected interface and controller signals.

The interface is generic and is not restricted to LindiCopter or LindiPlane.

Consult the local README for detailed information and instructions.
