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


## Tests

1. The simple wing implementation uses functions which are not solely inside the [simple_wing](#simple_wing) folder.
Thus, it is mandatory that you have access to the whole LADAC library. [Install LADAC according to
its README.](../../README.md)
2. Start a test computation by running `simpleWing_example.m` (located in the [test](test) folder):
    ```
    simpleWing_example
    ```
   Three examples are computed that generate two figures and one console output.
3. Start a test computation in Simulink:
	```
	simpleWing_example_init_simModel
	sim('simpleWing_example_simModel')
	```
   Discover the inputs and outputs of the Simulink block by opening the Scopes and reading the descriptions.


## How to use

- Read the wing aerodynamics chapter in [1] to understand the method.
- You can define your desired wing:
  - Make a copy of the `params_aero_simple_wing_default` template file (outside of LADAC)
  - Make yourself familiar with the required parameters as well as the structure and order of initializations and the parameter structs
  - Adjust the parameters in your copy of the template file
- In the test scripts in the [Tests](#Tests) section you can now exchange the `'params_aero_simple_wing_default'`
string with the filename of your copy
- Continue with step 2 of the [Tests](#Tests) section.


## Literature
[1] [Beyer, Y. (2017). Flight Control Design and Simulation of a Tandem Tilt Wing RPAS, Masterarbeit, Institute of Flight Guidance, TU Braunschweig.](https://github.com/iff-gsc/ladac/-/wikis/uploads/f25254ae09d10b999f34ad87b4e3e72f/2017_Beyer-Flight_Control_Design_and_Simulation_of_a_Tandem_Tilt_Wing_RPAS.pdf)
