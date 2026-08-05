# Getting started

This guide gives a new user a practical route into LADAC without requiring a detailed understanding of every source directory.

## 1. Install and verify LADAC

Follow [Installation](Installation.md), add LADAC to the MATLAB path, and run:

```matlab
check_ladac
```

Open the main Simulink library:

```matlab
open_system('ladac_lib')
```

The library browser provides access to the main modeling and control areas.

## 2. Start with LADAC-Examples

LADAC itself contains reusable functions, libraries, and component-level examples.
Complete executable aircraft simulations are maintained in [LADAC-Examples](https://github.com/iff-gsc/LADAC-Examples).

Clone it recursively:

```bash
git clone --recursive https://github.com/iff-gsc/LADAC-Examples.git
```

The current LADAC-Examples README documents a multicopter simulation that can be initialized and run in MATLAB/Simulink.
Other examples cover fixed-wing and eVTOL configurations.

## 3. Understand the LADAC pattern

Many LADAC components follow the same pattern:

- a parameter template such as `myProject_params_default.m`
- a loading or creation function such as `myProjectLoadParams` or `myProjectCreate`
- MATLAB functions implementing the underlying equations
- a Simulink library named after the source directory
- an example or unit test
- a local `README.md`

Do not modify the default parameter files for a specific project.
Copy them into the application repository and adapt the copy.

Example:

```matlab
% In your project repository:
my_wing = simpleWingLoadParams('my_wing_params');
```

The exact signature differs between components; consult the corresponding local README and function help.

For a broader overview of the framework layers and repository responsibilities, see [Architecture and workflows](Architecture.md).

## 4. Choose an entry point

### I want to simulate an aircraft

Read [Aircraft modeling](Aircraft_Modeling.md), then start from the closest complete model in LADAC-Examples.

### I want to inspect an aerodynamic model

Useful starting points include:

- [aerodynamics/simple_wing](../aerodynamics/simple_wing)
- [aerodynamics/vlm_wing](../aerodynamics/vlm_wing)
- [aerodynamics/airfoil](../aerodynamics/airfoil)
- [aerodynamics/fuselage](../aerodynamics/fuselage)
- [aerodynamics/rotorcraft](../aerodynamics/rotorcraft)

Run the example listed in the local README before modifying parameters.

### I want to develop a controller

Read [Flight control](Flight_Control.md).
Start with a component-level controller example or an existing LindiCopter/LindiPlane application in LADAC-Examples.

### I want to run ArduPilot against a LADAC aircraft model

Use the ArduPilot SITL interface in [interfaces/ArduPilot_SITL](../interfaces/ArduPilot_SITL) and follow its local README.

### I want to deploy my Simulink controller to ArduPilot

Read [ArduPilot integration](ArduPilot_Integration.md) and the local README in [interfaces/ArduPilot_custom_controller](../interfaces/ArduPilot_custom_controller).

## 5. Work in an application repository

For research and development projects, keep project-specific content outside LADAC:

```text
my_project/
├── init/
├── models/
├── parameters/
├── tests/
└── modules/
    └── LADAC/       # Git submodule
```

Project-specific aircraft parameters, controller tuning, simulations, data, and generated code belong in the application repository.
Reusable functionality that is independent of one aircraft or publication may be contributed back to LADAC.

## 6. Testing and troubleshooting

If something does not work, read the [Testing and troubleshooting](Testing.md) guide.
It also describes the distinction between execution tests, verification, validation, and system testing.

## 7. Record versions

For reproducibility, record at least:

- MATLAB release
- LADAC commit
- commits of LADAC submodules
- LADAC-Examples commit, where applicable
- ArduPilot commit and branch, where applicable
- required toolboxes and external software versions

## 8. Before flight testing

LADAC is research software.
A successful simulation is not sufficient evidence that a controller is safe for flight.

Before flight tests:

- test component behavior and sign conventions,
- test actuator limits and failure behavior,
- use software-in-the-loop simulation,
- verify that a safe standard flight mode can be selected,
- implement and test appropriate failsafe behavior,
- conduct tests only in a suitable area and under responsible supervision.

The local [ArduPilot interface](../interfaces/ArduPilot_custom_controller) documentation contains additional warnings and workflow details.
