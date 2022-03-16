# Unsteady aerodynamics model for airfoils

This project contains a reduced order unsteady aerodynamics model for airfoils with high computational efficiency.  
The implemented linear state-space model also takes into account compressibility effects and therefore depends on the Mach number.
There are three differently implemented functions, but all are according to [1] and return equal results.
Note that this model does not work for stall effects.  
If you want to simulate dynamic stall, please take a look at the [dynamic stall model](../dynamic_stall).

## Motivation

If you want to perform fast computations of unsteady aerodynamics, a reduced order model is required.
This model only requires a few parameters and returns good values below stall.
Note that there is no function for the unsteady drag coefficient yet (only lift and pitching moment).  
Please refer to the documentation of each [content](#contents) for more information.

## Installation

- You must install [LADAC](../../../README.md)


## Tests

Run the test script to get some first results and impressions of how it works:
   ```
   test_unstAirfoilAero
   ```

For more information, please take a look at the documentation of the test script, the documentation of the functions and the literature.

## Literature

[1] Leishman, J. G., and Nguyen, K. Q. (1990). State-space representation of unsteady airfoil behavior. AIAA journal, 28(5), 836-844.  
