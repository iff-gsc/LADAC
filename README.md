![LADAC](docs/images/ladac-logo.svg)

**L**ibrary for **A**ircraft **D**ynamics **A**nd **C**ontrol

LADAC is an open-source MATLAB/Simulink framework for aircraft modeling, flight dynamics, simulation, and flight-control development.

It provides modular models and development tools for applications ranging from small unmanned aircraft and multicopters to transport aircraft and flexible aeroelastic configurations. LADAC supports workflows from component-level modeling and controller development to complete nonlinear simulation, software-in-the-loop testing, code generation, and flight testing with ArduPilot.

LADAC has been developed and used in research projects involving fixed-wing UAVs, multicopters, eVTOL aircraft, transport aircraft, aeroelastic flight-dynamics models, active gust-load alleviation, and advanced flight-control methods.

> **Status:** LADAC is research software under active development. The maturity, validation status, software requirements, and supported configurations differ between individual components. Consult the corresponding module documentation before relying on a model or controller for a particular application.

## Key features

### Modular aircraft and subsystem modeling

- Complete and subsystem-level models for fixed-wing aircraft, multicopters, and eVTOL configurations
- Rigid-body and flexible-body equations of motion
- Modular aerodynamic, propulsion, actuator, sensor, atmosphere, turbulence, gust, and ground-contact models
- Parameter-based aircraft definitions for reusable and configurable simulation models
- Support for suspended payloads and multi-body configurations

### Aerodynamic modeling

- Nonlinear finite-wing model intended for large angles of attack and sideslip
- Vortex-lattice-based wing model with viscous airfoil coupling
- Aerodynamic derivative, downwash, distributed-load, and bending-moment calculations
- Steady and unsteady airfoil aerodynamics, dynamic stall, and flap-effect models
- Rigid and aeroelastic wing configurations
- Fuselage, propeller, rotorcraft, and induced-velocity models

### Flight dynamics and aeroelasticity

- Quaternion-based rigid-body equations of motion
- Flexible-body models based on structural modes
- Structural model creation, reduction, load transfer, and visualization utilities
- Coupling of aerodynamic and structural models
- Atmospheric turbulence and discrete-gust models

### Guidance, control, and autopilots

- Nonlinear dynamic inversion (NDI) and incremental nonlinear dynamic inversion (INDI)
- Control-effectiveness modeling and constrained control allocation
- Reusable attitude, acceleration, altitude, position, and trajectory-control modules
- Flight modes for attitude control, altitude hold, loiter, and trajectory following
- Trajectory generation and waypoint navigation
- Linear-systems, filtering, and controller-development utilities

LADAC includes two configurable INDI-based autopilots:

- **[LindiCopter](control/autopilots/LindiCopter/README.md)** — a modular multicopter autopilot with attitude, altitude, position, and flight-mode logic
- **[LindiPlane](control/autopilots/LindiPlane/README.md)** — a modular fixed-wing autopilot currently focused on conventional aircraft configurations

Both autopilots provide functions that derive controller parameter structures from the corresponding aircraft-model parameters.

### Simulation, visualization, and external interfaces

- MATLAB/Simulink-based nonlinear simulation
- [ArduPilot software-in-the-loop interface](interfaces/ArduPilot_SITL/README.md)
- [Deployment of generated MATLAB/Simulink controllers to ArduPilot](interfaces/ArduPilot_custom_controller/README.md)
- Processing of generated controller parameters as ArduPilot parameters
- Logging of controller interface signals through ArduPilot
- FlightGear and FlexiFlightVis visualization interfaces
- CPACS/TiGL and Tornado interfaces

## Getting started

For a first installation and simulation:

1. Read the [installation guide](docs/Installation.md).
2. Follow the [getting-started guide](docs/Getting_Started.md).

## Typical workflows

LADAC supports several complementary workflows:

- **Aircraft simulation:** assemble nonlinear aircraft models from reusable subsystem blocks.
- **Flight-control development:** design and assess controllers using the same aircraft parameterization as the simulation.
- **Aeroelastic research:** couple unsteady aerodynamic and structural models for flexible-aircraft studies.
- **ArduPilot SITL:** run ArduPilot against a LADAC aircraft-dynamics model.
- **Controller deployment:** generate C++ code from a Simulink controller and run it in a custom ArduPilot flight mode.

See [Architecture and workflows](docs/Architecture.md) for an overview.

## Documentation

| Topic | Documentation |
|---|---|
| First steps | [Getting started](docs/Getting_Started.md) |
| Software requirements and setup | [Installation](docs/Installation.md) |
| Framework structure and workflows | [Architecture](docs/Architecture.md) |
| Building aircraft simulations | [Aircraft modeling](docs/Aircraft_Modeling.md) |
| Control modules and autopilots | [Flight control](docs/Flight_Control.md) |
| ArduPilot code-generation workflow | [ArduPilot integration](docs/ArduPilot_Integration.md) |
| Tests and troubleshooting | [Testing](docs/Testing.md) |
| Contributing code | [Development guide](docs/Development.md) |
| Commit conventions | [Rules for commits](docs/Rules_for_commits.md) |

Detailed documentation for individual components is located in the corresponding source directories.

## Repository structure

| Directory | Contents |
|---|---|
| [aircraft](aircraft) | Complete aircraft models and aircraft subsystem assemblies |
| [actuators](actuators) | Actuator dynamics and command conversions |
| [aerodynamics](aerodynamics) | Airfoil, wing, fuselage, rotorcraft, and downwash models |
| [control](control) | Control methods, reusable controller modules, guidance, flight modes, and autopilots |
| [environment](environment) | Atmosphere, turbulence, gust, and ground-contact models |
| [equations_of_motion](equations_of_motion) | Rigid-body, flexible-body, and structural dynamics |
| [flight_parameters](flight_parameters) | Derived flight-state and flight-path quantities |
| [interfaces](interfaces) | Interfaces to ArduPilot, FlightGear, FlexiFlightVis, TiGL, Tornado, and other tools |
| [propulsion](propulsion) | Propeller, motor, and battery models |
| [sensors](sensors) | Sensor models |
| [utilities](utilities) | Mathematical, coordinate-transformation, quaternion, and general helper functions |
| [modules](modules) | Separately maintained dependencies included as Git submodules |
| [external](external) | Third-party code distributed with LADAC |
| [docs](docs) | User and developer documentation |

## Examples

Complete executable examples are maintained in the separate [LADAC-Examples](https://github.com/iff-gsc/LADAC-Examples) repository. It currently contains UAV examples for multicopters, fixed-wing aircraft, and eVTOL configurations using rigid-body flight-dynamics models and the LindiCopter or LindiPlane autopilots.

Research-specific models and code associated with publications are generally maintained in separate repositories that include LADAC as a Git submodule.

## Contributing

Bug reports, documentation improvements, new models, and code contributions are welcome.

Before contributing:

1. Read the [development guide](docs/Development.md).
2. Follow the [commit-message rules](docs/Rules_for_commits.md).
3. Search the existing [issues](https://github.com/iff-gsc/LADAC/issues).
4. Run the relevant tests and `check_ladac`.

## License

LADAC is licensed under the [GNU General Public License v3.0](LICENSE).

Individual components in `modules` and `external` may use different licenses. Consult the license files and documentation shipped with each dependency.
