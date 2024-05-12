# Simple wing

Simple wing is a 2 point model implementation of a wing that is especially intended to 
provide reasonable results for high angles of attack or sideslip angles.


## Motivation 

Flight dynamics models often contain aerodynamic derivatives as aerodynamics model.
However, these models are usually almost linear and are consequently only valid
for specific operating points. This problem can be compensated by using lookup tables
for the aerodynamic derivatives so that appropriate derivatives can be interpolated
depending on the operating point. However, this solution still has some disadvantages like
performance problems in case of large tables, considerable amount of work to generate the tables
and difficulties of computing good derivatives for high angles of attack and sideslip angles.
Especially vertical take-off and landing airplanes usually require such aerodynamics models
as they operate at high angles of attack and sideslip angles.  
The simple wing implementation tries to solve this problem with a 2 point implementation.
A wing is simplified to have 2 points (one at the left and one at the right) where the
aerodynamics are computed. The user has to specify geometric parameters as well as
a lift and drag coefficient polar for the wing. Aerodynamic moments are computed by
lift and drag forces that are applied at the 2 points.
The lift due to a flap is modeled by a simple analytic model.  
This method is explained in [1].

## Installation

The simple wing implementation uses functions which are not solely inside the [simple_wing](#simple_wing) folder.
Thus, it is mandatory that you have access to the whole LADAC library. [Install LADAC according to
its README.](../../README.md)

## Examples

1. Start a example computations by running `simpleWing_example.m` (located in the [examples](examples) folder):
    ```
    simpleWing_example
    ```
   Three examples are computed that generate two figures and one console output.
2. Start an example computation in Simulink:
	```
	simpleWing_example_init_simModel
	sim('simpleWing_example_simModel')
	```
   Discover the inputs and outputs of the Simulink block by opening the Scopes and reading the descriptions.
3. Run an example where the main aerodynamic derivatives at small aerodynamic angles are corrected by a vortex lattice method:
    ```
    simpleWing_example_vlm
    ```

## How to use

- Read the wing aerodynamics chapter in [1] to understand the method.
- You can define your desired wing:
  - Make a copy of the `simpleWing_params_default` template file (outside of LADAC) and load the parameters using the function `simpleWingLoadParams` (see example 1).
  - You can also specify the geometric parameters based on the `wing_params_default` template file using the [vortex lattice method of LADAC](../vlm_wing) and load the parameters using the function `simpleWingCreate` (see example 3).
  - Make yourself familiar with the required parameters as well as the structure and order of initializations and the parameter structs.
  - Adjust the parameters in your copy of the template file.
- In the example scripts in the [Example](#Example) section you can now exchange the `'simpleWing_params_default'` and `'wing_params_default'`
strings with the filenames of your copies.
- For use with Simulink take a look at the 2nd example in the [Example](#Example) section.
- For use in Matlab take a look at the `simpleWingRun` function. This function is similar to the Simulink block `wing 2 point aerodynamics` in the `simple_wing_lib.slx`.


## Literature
[1] [Beyer, Y. (2017). Flight Control Design and Simulation of a Tandem Tilt Wing RPAS, Masterarbeit, Institute of Flight Guidance, TU Braunschweig.](https://github.com/iff-gsc/ladac/-/wikis/uploads/f25254ae09d10b999f34ad87b4e3e72f/2017_Beyer-Flight_Control_Design_and_Simulation_of_a_Tandem_Tilt_Wing_RPAS.pdf)
