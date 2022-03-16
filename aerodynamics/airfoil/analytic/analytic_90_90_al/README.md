# Analytic airfoil coefficients for AoA from -90° to 90° (self-developed)

The model can yield good results for angles of attack between -90°
and 90°.
However, the automatic curve fitting may be less robust than for other projects with a smaller range of angles of attack.
Moreover, this implementation is not ready for dynamic stall models yet.

## Tests

Run the test script to get some first results and impressions of how it works:
   ```
   example_airfoilAnalytic9090
   ```
   
## How it works

If the input data does not contain high or low angles of attack, data points will be added automatically to improve the chances of a good automatic curve fitting.
For more information, please take a look at the function documentation as well as the literature stated in the function documentation.