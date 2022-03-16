# Dynamic stall model for airfoils

This project contains a reduced order dynamic stall model for airfoils with high computational efficiency.
It is based on the Beddoes-Leishman dynamic stall model with state-space implementation [1], [2] but with some modifications.
This dynamic stall model works approximately in the range of angles of attack from -5° to 15°.

## Motivation

If you want to perform fast computations of dynamic stall, a reduced order model is required.
Such models are typically based on static airfoil coefficients. Typically analytic functions, fitted to the static coefficients, are used.  
Moreover, typically the flow separation point is used as additional state.
That is why the static airfoil coefficients usually must be a function of the flow separation point.  
Since the analytic function for the lift coefficient was not really satisfactory, the analytic model [Analytic airfoil: AoA from -5° to 15°](../analytic/analytic_0515_alpha) was used instead.
The approach of [3] was used to compute the flow separation point afterwards.
The drag coefficient implementation was also based on [Analytic airfoil: AoA from -5° to 15°](../analytic/analytic_0515_alpha) and [3].
But note that for the pitching moment coefficient the function from [1] and [2] was not modified.  
Also note that the drag coefficient implementation should be improved.  
Please refer to the documentation of each [content](#contents) for more information.

## Installation

- You must install [LADAC](../../../README.md)


## Tests

Run the test script to get some first results and impressions of how it works:
   ```
   test_airfoilDynStall
   ```

For more information, please take a look at the documentation of the test script, the documentation of the functions and the literature.

## Literature
[1] Leishman, J. (1989). State-space model for unsteady airfoil behavior and dynamic stall. In 30th Structures, Structural Dynamics and Materials Conference (p. 1319).  
[2] Leishman, J. G., & Beddoes, T. S. (1989). A Semi-Empirical model for dynamic stall. Journal of the American Helicopter society, 34(3), 3-17.  
[3] Hansen, M. H., Gaunaa, M., & Madsen, H. A. (2004). A Beddoes-Leishman type dynamic stall model in state-space and indicial formulations.  
