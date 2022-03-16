# Simple analytic airfoil coefficients

The model can yield good results for angles of attack between -10°
and 10° and does not include any stall effects.  
Lift and pitching moment coefficient are linear functions (two parameters each).
The drag coefficient is a quadratic function (three parameters).

## Tests

Run the test script to get some first results and impressions of how it works:
   ```
   test_airfoilAnalyticSimple
   ```
   
## How it works

Please take a look at the function documentation as well as the parameters file template `airfoilAnalyticSimple_params_default`.
