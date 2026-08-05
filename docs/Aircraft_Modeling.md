# Aircraft modeling

## Overview

LADAC supports component-level calculations and complete nonlinear aircraft simulations.
Models are assembled from reusable physical subsystems, equations of motion, aircraft-specific parameter structures, and optional control or visualization components.

The available implementations differ in fidelity, scope, and validation status.
Select models according to the research question rather than assuming that the most detailed implementation is always the best choice.

## Complete aircraft models

The [aircraft](../aircraft) directory contains complete and subsystem-level assemblies for:

- conventional fixed-wing aircraft,
- quadcopters and multicopter subsystems,
- eVTOL configurations including tailsitter and tandem-tiltwing structures,
- suspended payloads.

These assemblies use lower-level models from the rest of LADAC.
Project-specific aircraft parameter files should normally be stored outside LADAC.

## Aerodynamic models

### Simple wing

[aerodynamics/simple_wing](../aerodynamics/simple_wing) implements a two-point finite-wing model intended to provide reasonable behavior at high angles of attack and sideslip.
Geometry and aerodynamic polars are specified by the user, and forces at two representative wing points generate the resulting forces and moments.

Typical uses include:

- nonlinear rigid-body flight-dynamics simulation,
- takeoff and landing,
- stall and unusual attitudes,
- eVTOL configurations with strongly varying inflow directions,
- simulations where computational cost should remain low.

The model can use small-angle derivative information obtained from the VLM model.

### VLM wing

[aerodynamics/vlm_wing](../aerodynamics/vlm_wing) implements a spanwise-discretized vortex-lattice method coupled with viscous two-dimensional airfoil data.

It supports, depending on configuration:

- spanwise circulation and aerodynamic coefficient distributions,
- complex wing geometry,
- aerodynamic derivatives,
- downwash,
- distributed loads and bending moments,
- steady and unsteady aerodynamic calculations,
- coupling to modal structural models for aeroelastic simulation.

The implementation currently uses spanwise discretization with one chordwise panel.
Consult its README and source references when assessing applicability.

### Airfoil models

[aerodynamics/airfoil](../aerodynamics/airfoil) contains:

- analytic steady airfoil models,
- airfoil data and loading utilities,
- unsteady aerodynamic models,
- dynamic-stall models,
- flap-effect models,
- selected micro-tab models.

The airfoil database includes multiple sources and formats.
Check the origin, Reynolds-number assumptions, flap configuration, and licensing of the selected data before use.

### Fuselage models

LADAC includes both [aerodynamics/simple_fuselage](../aerodynamics/simple_fuselage) and a more detailed [aerodynamics/fuselage](../aerodynamics/fuselage) implementation.
The latter contains geometry, local inflow, unsteady aerodynamic, structural, and CPACS-related functionality.
The model choice should be based on the required fidelity and the available geometry and aerodynamic data.

### Rotorcraft aerodynamics

[aerodynamics/rotorcraft](../aerodynamics/rotorcraft) includes induced-velocity models and blade-element-method functions.
The induced-velocity area includes baseline momentum-theory functions and extensions associated with angle of attack and vortex-ring-state behavior.

### Downwash and interaction

[aerodynamics/wing_downwash](../aerodynamics/wing_downwash) and common aerodynamic functions provide selected methods for aerodynamic interaction and downwash calculations.

## Equations of motion

### Rigid body

[equations_of_motion/rigid_body](../equations_of_motion/rigid_body) contains quaternion-based six-degree-of-freedom kinetics and kinematics.
It uses forward-right-down body axes and north-east-down geodetic axes according to the conventions documented in the local README.

### Flexible body and structure

[equations_of_motion/flexible_body_simple](../equations_of_motion/flexible_body_simple) and [equations_of_motion/structure](../equations_of_motion/structure) provide modal flexible-body dynamics and structural utility functions.
Available operations include model creation from Nastran data, model reduction, mass and inertia calculations, load-vector generation, load transformations, scaling, and eigenmode visualization.

Aeroelastic models can couple these structural states to the `vlm_wing` and `fuselage` aerodynamic models.

## Propulsion and energy

[propulsion](../propulsion) contains:

- map-based propeller models,
- motor-related models and functions,
- battery parameterization and discharge behavior.

Check whether a given model is static, quasi-static, or dynamic and whether the source data cover the intended operating range.

## Actuators, sensors, and environment

- [actuators](../actuators) contains command conversion and actuator-dynamics models.
- [sensors](../sensors) contains selected sensor output models and is currently less comprehensive than several other LADAC areas.
- [environment](../environment) includes atmospheric properties, turbulence, discrete gusts, and ground-contact models.

## Recommended model-building workflow

1. Start from the closest example or complete aircraft assembly.
2. Copy aircraft-specific parameter templates into the application repository.
3. Define mass, inertia, geometry, actuator, propulsion, and aerodynamic data.
4. Run component-level examples and tests.
5. Assemble and trim the complete aircraft model.
6. Compare steady-state values and derivatives with independent data.
7. Validate transient behavior against higher-fidelity calculations or measurements.
8. Record the assumptions, source data, LADAC commit, and known validity range.

## Validation

LADAC does not currently provide one uniform validation level for all components.
For each project, document:

- model source and literature,
- parameter-identification method,
- comparison data,
- tested operating envelope,
- numerical settings,
- known extrapolation,
- uncertainties and limitations.

A local component example demonstrates operation; it does not automatically establish validation for a new aircraft.
