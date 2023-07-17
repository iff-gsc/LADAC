# FlexiFlightVis: Aeroelastic Flight Dynamics Visualization

Since Matlab/Simulink has limited capabilities in 3D visualization, you can
use FlexiFlightVis, a program written in C++, to visualize aeroelastic airplane simulations in real-time.


## Installation
- Install [LADAC](https://github.com/iff-gsc/LADAC#readme)
- Install [FlexiFlightVis](https://github.com/iff-gsc/FlexiFlightVis)


## Example

1. Initialize the Simulink example model:  
    ```
    flexiFlightVis_lib_example_init;
    ```
2. Run the Simulink example model:
    ```
    sim('flexiFlightVis_lib_example')
    ```
3. Run FlexiFlightVis.  
  You should now see a fuselage and 2 wings in FlexiFlightVis.
  In this simple example the airplane geometry is constant (thus rigid) and the aerodynamic model is not updated.
  That is why no aerodynamic forces are displayed.


## How to use

In the Simulink library block `Send to FlexiFlightVis` you can specify the sample time, IP address (of FlexiFlightVis computer) as well as cell arrays of wing.state and fuselage.state structs.
These structs should be initialized as in the initialization script `flexiFlightVis_lib_example_init`.
For simulation of the wing aerodynamics and fuselage aerodynamics you should use blocks from the LADAC libraries `vlm_wing_lib` and `fuselage_lib`.