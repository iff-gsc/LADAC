# Unsteady or quasisteady flap coefficients

This project contains a reduced order unsteady aerodynamics model for airfoils flaps with high computational efficiency according to [1].    
Note that the implementation of this model is not complete yet. Neither a drag coefficient nor a pitching moment coefficient are modeled. Also note that the lift coefficient does not consider the noncirculatory part of the lift yet.

## Motivation

If you want to perform fast computations of unsteady flap aerodynamics, a reduced order model is required.
This model only requires a few parameters and returns good values below stall.
Usually, this model is additive to the [unsteady airfoil model](../unsteady) or the [dynamic stall model](../dynamic_stall).  
Please refer to the documentation of each [content](#contents) for more information.

## Installation

- You must install [LADAC](../../../README.md)


## Example

Run the example script to get some first results and impressions of how it works:
   ```
   airfoilFlapUnst_example
   ```

For more information, please take a look at the documentation of the example script, the documentation of the functions and the literature.

## Literature

[1] Leishman, J. G. (1994). Unsteady lift of a flapped airfoil by indicial concepts. Journal of Aircraft, 31(2), 288-297.  
