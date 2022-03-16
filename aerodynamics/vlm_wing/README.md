# Wing Aerodynamics with Vortex Lattice Method and Viscous Coupling

This program calculates the lift- drag- and momentum contribution for a specified wing 
according to the vortex lattice method coupled with viscous 2D profile aerodynamics.
The computation is based on a specified wing geometry that is discretized
into multiple wing strips. Note that this implementation currently only supports spanwise discretization
so that there is always only one chordwise panel. For each strip predefined 2D airfoil 
aerodynamics maps are used. Each computation requires an iterative computation
until convergence in case of steady computations. This project also supports unsteady
computations based on 2D unsteady state space models for subsonic Mach numbers.
It is also possible to couple the aerodynamics model with a structural model which
yields and aeroelastic model.

## Motivation 

For flight dynamics computations high fidelity aerodynamics models are too
computation-expensive. However, low fidelity aerodynamics models like
the lifting line theory are not accurate enough or can not provide
a spanwise distribution of the aerodynamic coefficients for complex wing geometries.
This implementation is a medium fidelity implementation of wing aerodynamics
which provides spanwise distribution of the aerodynamic coefficients for
complex wing geometries with moderate accuracy and relatively low
computational cost.
This method is based on [1], [2] and [3].


## Installation

- You need [LADAC](../../README.md)


## Tests

### General

Start a test computation by running `wing_example.m` (located in the [test](test) folder):
  ```
  wing_example
  ```
Two figures should appear: spanwise lift coefficient distribution and geometric visualization of the wing.

### Simulink

Initialize the wing struct, the wing state bus object and the atmosphere bus object (a Simulink file will open and will be run):
  ```
  wing_example_sim_init
  ```
The above test is for a rigid wing with steady aerodynamics. To test a rigid wing with unsteady aerodynamics, run:
  ```
  wing_example_sim_unst_init
  ```

## How to use

### General
- You can define your desired wing:
  - Take a look at `wingCreate`. You will see that you need to specify a parameters file. Therefore you should use the `wing_params_default` as a template.
  - Make a copy of `wing_params_default` and name it `wing_params_mywing`.
  - Adjust the parameters according to the documentation/comments.
  - Now open the script `wing_example.m` located in the [test](test) folder and change the input variables of the function call `wingCreate`. Amend it to the name given to your parameter file.
  - Run `wing_example` again and you will see the geometry of your wing as well as a spanwise lift distribution.
  - For more information, please study the script `wing_example`.

### Simulink

There a four Simulink blocks based on the same core:  
1. The block `wing vlm rigid steady` considers a rigid wing and steady aerodynamics.
2. The block `wing vlm rigid unsteady` considers a rigid wing and unsteady aerodynamics.
3. The block `wing vlm flexible steady` considers a flexible wing and unsteady aerodynamics.
4. The block `wing vlm aeroelastic` considers a flexible wing and unsteady aerodynamics.

The inputs and outputs of these blocks are described below. 

**Inputs/parameters:**

Variable | Explanation
--- | ---
alpha_K | flight-path angle of attack (scalar), in rad
beta_K | flight-path sideslip angle (scalar), in rad
V_A | airspeed (scalar), in m/s
omega_Kb | angular velocity of the wing relative to the earth (3x1 array), in rad/s
atmosphere | Simulink bus based on the isAtmosphere function
actuators_pos | actuator positions (1xN_a array, where N_a is the number of actuators), usually in deg
actuators_rate | actuator rates (1xN_a array), usually in deg/s
incidence | incidence angle of the wing frame with respect to the aircraft frame (scalar), in rad 
xyz_cg_wing | center of gravity position in the wing frame (3x1 array), in m 
V_Kb_dt | time-derivative of the velocity of the center of gravity relative to the earth (3x1 array), in m/s^2 
omega_Kb_dt | time-derivative of omega_Kb (3x1 array), in rad/s^2 
structure_state | aircraft structure state vector ((2*N_m array)x1 array, where N_m is the number of mode shapes), where the first half is the position and the second half the velocity
structure_accel | aircraft structure acceleration vector (N_mx1 array)
V_ext_local | local wind vectors (3xN_p array, where N_p is the number of panels), in m/s
V_ext_local_dt | time-derivative of V_ext_local, in m/s^2
unst_aero_state | unsteady aerodynamics state vectors (8xN_p array), this should be integrated from wing_state.aero.unsteady.x_dt
dyn_stall_state | dynamic stall state vectors (3xN_p array), this should be integrated from wing_state.aero.unsteady.X_dt
unst_flap_state | unsteady flap aerodynamics state vectors (2xN_p array), this should be integrated from wing_state.aero.unsteady.z_dt
unst_act2_state | unsteady aerodynamics state vectors of the second actuator (2xN_p array), this should be integrated from wing_state.aero.unsteady.z2_dt
tau_v | dimensionless time of the leading edge vortex of the dynamic stall model (1xN_p array), this should be integrated from wing_state.aero.unsteady.tau_v_dt
Delta_alpha | angle of attack corrections for the VLM to match with the airfoil (1xN_p array), this should be fed back from wing_state.aero.circulation.Delta_alpha with a unit delay to speed up convergence (similar results)
alpha_ind | induced angles of attack (1xN_p array), this should be fed back from wing_state.aero.circulation.alpha_ind with a unit delay to speed up convergence (similar results)

**Outputs:**

The output is a Simulink bus based on the `wing.state` struct.
For the definition of the `wing.state` struct, have a look at `wingCreateState`.
For more detailed information you may have to look inside the code starting with `wingSetState`.

## Literature
[1] Barnes, J. P. (1997). Semi-empirical vortex step method for the lift and induced drag loading of 2D and 3D wings. SAE Paper 975559.

[2] Van Dam, C. P. (2002). The aerodynamic design of multi-element high-lift systems for transport airplanes. Progress in Aerospace Sciences, 38(2), 101-144.

[3] Goitia, H., & Llamas, R. (2019). Nonlinear vortex lattice method for stall prediction. In MATEC Web of Conferences, 304, 02006.
