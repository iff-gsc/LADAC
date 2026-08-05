# Installation

## Requirements

LADAC is developed for MATLAB and Simulink. The currently documented baseline is:

- MATLAB and Simulink R2023b or later

Newer releases may work, but compatibility is not guaranteed for every component.
Older MATLAB releases may require exporting Simulink files to an older format.

Additional products are required only for particular workflows.
Depending on the components used, these may include:

- MATLAB Coder
- Simulink Coder
- Embedded Coder
- Curve Fitting Toolbox
- MATLAB Compiler

The exact requirements are not yet recorded centrally for every LADAC component.
If MATLAB reports a missing product, consult the local component documentation and the referenced functions.

Optional external software includes:

- FlightGear for visualization
- ArduPilot SITL for software-in-the-loop simulation
- QGroundControl, Mission Planner, or MAVProxy for interaction with ArduPilot
- TiGL/TiXI for selected CPACS-based geometry workflows
- Tornado for the corresponding aerodynamic interface

## Clone LADAC

Clone LADAC together with its Git submodules:

```bash
git clone --recursive https://github.com/iff-gsc/LADAC.git
```

If LADAC has already been cloned without submodules:

```bash
cd LADAC
git submodule update --init --recursive
```

For a research or application repository, LADAC should normally be included as a Git submodule rather than copied:

```bash
git submodule add https://github.com/iff-gsc/LADAC.git modules/LADAC
git submodule update --init --recursive
```

Pinning LADAC to a specific commit is recommended for reproducible research and long-lived projects.

## Add LADAC to the MATLAB path

In MATLAB:

```matlab
addpath(genpath('/path/to/LADAC'))
```

To make the path persistent, use `savepath` or add the command to a project-specific initialization script.
A project-specific script is usually preferable because it keeps the required LADAC version and path explicit.

Avoid adding multiple LADAC checkouts to the MATLAB path at the same time.

## Verify the installation

Run:

```matlab
check_ladac
```

Then verify the Simulink libraries:

1. Open Simulink.
2. Open the Library Browser.
3. Refresh the browser if necessary.
4. Check that `LADAC` appears at the top level.
5. Open several sublibraries and verify that blocks can be inserted into a model.

The main library can also be opened directly:

```matlab
open_system('ladac_lib')
```

## Install LADAC-Examples

The easiest way to explore complete models is the separate LADAC-Examples repository:

```bash
git clone --recursive https://github.com/iff-gsc/LADAC-Examples.git
```

LADAC-Examples includes LADAC and additional example data as submodules.
Follow its README for the currently available simulations.

## Updating

When LADAC is used as a submodule, update deliberately and test the project after changing the pinned commit:

```bash
cd modules/LADAC
git fetch
git checkout <desired-commit-or-tag>
cd ../..
git add modules/LADAC
git commit
```

After switching LADAC versions, run:

```bash
git submodule update --init --recursive
```

and execute the LADAC and project-specific tests.

## Version compatibility

Some interfaces depend on matching versions in multiple repositories.
This is particularly important for the generated MATLAB/Simulink controller interface to the ArduPilot fork.
Use versions or commits that are documented as compatible; otherwise, interface variables, generated files, or parameter handling may differ.

A formal compatibility matrix is not yet part of LADAC.
Until one is introduced, record the exact LADAC, LADAC-Examples, MATLAB, and ArduPilot commit identifiers in each project.
