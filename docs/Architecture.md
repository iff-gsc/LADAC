# Architecture and workflows

## Scope

LADAC is a modular MATLAB/Simulink framework rather than one monolithic aircraft simulator.
Its components can be used individually or assembled into complete nonlinear simulations and flight-control development environments.

The framework covers four broad layers:

1. physical and environmental component models,
2. complete aircraft and flight-dynamics models,
3. guidance, control, and autopilot functions,
4. interfaces to visualization, simulation, and flight software.

## Main layers

```text
Aircraft definition and parameters
                |
                v
Physical subsystem models
(aerodynamics, propulsion, actuators,
 environment, sensors, structures)
                |
                v
Equations of motion and complete aircraft
                |
                v
Guidance, control, flight modes, autopilots
                |
                v
Simulation, visualization, SITL, code generation
                |
                v
ArduPilot and flight testing
```

The diagram represents common workflows, not a strict dependency graph.
For example, an aerodynamic model may be used independently for derivative calculations, and a controller module may be tested with a simple mass-point model.

## Source-tree responsibilities

### Physical models

- `aerodynamics`
- `propulsion`
- `actuators`
- `environment`
- `sensors`

These directories contain physical component models and supporting calculations.

### Dynamics

- `equations_of_motion`
- `flight_parameters`

These directories implement rigid-body, flexible-body, structural, kinematic, and derived flight-state calculations.

### Aircraft assemblies

- `aircraft`

This directory combines lower-level models into complete vehicle models and reusable aircraft subsystems.
Current categories include airplanes, multicopters, eVTOL aircraft, and suspended payloads.

### Guidance and control

- `control`

This directory includes control methods, control allocation, reusable controller modules, flight modes, trajectory and waypoint-navigation functions, filters, and the LindiCopter/LindiPlane autopilots.

### External interfaces

- `interfaces`

This directory connects LADAC to external software and runtime environments, including ArduPilot SITL, generated controllers in ArduPilot, FlightGear, FlexiFlightVis, TiGL/CPACS, and Tornado.

### Shared code and dependencies

- `utilities` contains LADAC-owned general-purpose functions.
- `modules` contains separately maintained dependencies included as Git submodules.
- `external` contains third-party code distributed with LADAC.

## MATLAB and Simulink implementations

A recurring LADAC design principle is to implement equations in MATLAB functions and expose them through linked Simulink library blocks.

This has several intended benefits:

- equations can be used directly from MATLAB,
- textual function changes can be reviewed with version-control tools,
- the same implementation can support component tests and Simulink models,
- code-generation-compatible functions can be reused for embedded deployment.

More complex components may use parameter and state structures, initialization functions, Simulink buses, and masked library blocks.

## Parameter flow

Many components separate default templates from initialized model structures:

```text
Parameter template
        |
        v
Load/create function
        |
        v
Validated or derived parameter structure
        |
        +----------------------+
        |                      |
        v                      v
MATLAB calculations       Simulink blocks
```

LindiCopter and LindiPlane extend this pattern by deriving autopilot parameter structures from corresponding aircraft-model parameters.
This reduces duplicate definition of vehicle properties, although controller tuning and configuration-specific inputs may still be required.

## Examples and research repositories

The repository roles are:

- **LADAC:** reusable framework components.
- **LADAC-Examples:** complete UAV examples demonstrating combinations of LADAC components.
- **Research repositories:** publication- or project-specific models, controller configurations, analyses, and data, normally including LADAC as a pinned submodule.

This separation keeps project-specific parameter sets and experimental code out of the framework while allowing generally reusable improvements to be contributed back.

## Documentation architecture

Cross-cutting workflows are documented in `docs/`. Component details remain beside the code. A component README should document:

- purpose and motivation,
- model assumptions and validity,
- inputs, outputs, parameters, and states,
- setup and examples,
- tests and validation status,
- known limitations,
- scientific references.
