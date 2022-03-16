# Rotorcraft Aerodynamics

This branch offers a set of functions to calculate the behavior of a rotor disk. It is based on the momentum theory [1] and uses model for the Vortex Ring State (VRS) [2], that is based on real rotor test data. The VRS model uses third-order polynomials, that connects stepwise the two branches of climb and fast descent. This VRS model creates an instability for the vortex ring state. Within the defined boundary the real part of the eigenvalue of heave damping become positive.

The calculations consider the influence of forward flight with the well known formulation of Glauert as well as the climb speed of the rotor plane.
For the power and torque calculation the profile drag is considered with a 2nd order model for the drag coefficient is respect to the incidence attack. [1]

The simulink models are able to simulate the rotor disk either steady-state in respect to the blade element momentum method (BEM), or with non-stationary induced inflow. [3]

The package includes functions to calculate the thrust and torque from given collective pitch angle and the velocity and dimensions of the rotor, as well as calculate the needed collective pitch angle from thrust. This can be used in a control system for dynamic inversion of the rotor system. This feature is also included for non-stationary induced inflow. [3]

For collective pitch quadcopters an easy-to-use control allocation function exists, too. [3]

For dynamic simulation also the dynamic derivatives of heave-damping Z_w and the control sensitivty Z_Theta can be calculated. [3], [4]

## Get started

1. The rotorcraft aerodynamic implementation uses functions which are not solely inside the rotorcraft_aerodynamics folder. Thus, it is mandatory
that you have access to the whole LADAC library. [Install LADAC according to
its README.](../../README.md)
2. In MATLAB, navigate to the LADAC directory and then go into the subdirectory of the rotorcraft aerodynamics implementation _aerodynamics/rotorcraft_aerodynamics_:
    ```
    cd('aerodynamics/rotorcraft_aerodynamics')
    ```
3. Start the computation by running `runRotorcraftAerodynamicsDemo.m`
    ```
    runRotorcraftAerodynamicsDemo
    ```
4. After the computation you should see the results in the command window.

## Unit tests

After any changes to the code, run the unit test to check if the code is still working as expected.

1.  In MATLAB, navigate to the LADAC directory and then go into the subdirectory of the rotorcraft aerodynamics implementation _aerodynamics/rotorcraft_aerodynamics_:
    ```
    cd('aerodynamics/rotorcraft_aerodynamics')
    ```
2.  Run all tests in the folder _aerodynamics/rotorcraft_aerodynamics_
    ```
    runtests('IncludeSubfolders',true)
    ```
3. Verify that all checks have passed


## Literature

[1] Van der Wall, B. G. (2015): Grundlagen der Hubschrauber-Aerodynamik. Springer.
 
[2] Johnson, W. (2005): Model for Vortex Ring State Influence on Rotorcraft Flight Dynamics. NASA Ames Research Center.

[3] Gücker, F. (2020): Lage- und Bahnregelung eines drehzahlsynchronen vier-rotorigen Drehflüglers mit kollektiver Blattverstellung. Master-Thesis. TU Braunschweig.

[4] Padfield, Gareth D. (2018): Helicopter Flight Dynamics. Third edition. Wiley.
