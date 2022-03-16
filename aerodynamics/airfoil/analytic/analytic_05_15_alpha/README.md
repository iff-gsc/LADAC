# Analytic airfoil coefficients for AoA from -5° to 15° (self-developed)

The model can yield good results for angles of attack between -5°
and 15° especially for high Mach numbers and thick cambered airfoils.
The implementation is based on multiple publications and can be used for the Beddoes-Leishman dynamic stall model.
The aerodynamic coefficients due to actuator states are currently just added to the clean airfoil coefficient (Delta coefficients).
For different Mach numbers, different analytic function parameters will be obtained.
The Mach number dependent function parameters should either be interpolated for different Mach numbers or fitted by a shallow neural network (which is faster, once it is trained).

## Tests

Run the test script to get some first results and impressions of how it works:
   ```
   example_airfoilAnalytic0515Al
   ```
   
The following test script shows how to fit the Mach number dependent function parameters with a neural network:
   ```
   example_airfoilAnalytic0515NeuN
   ```
   
## How it works

The analytic airfoil model using the neural network requires some parameters (maximum absolute function parameters for scaling and the neural network weights as well as number of inputs and outputs).
These parameters should be stored in a parameters file.
For the F15 airfoil from DLR there is a parameter file: `airfoilAnalytic0515_params_F15`.
Using this parameters file convention, the parameters should be loaded using the function `airfoilAnalytic0515LoadParams`
The following test script shows that the neural network based computation of the Mach dependent function parameters is faster than the interpolation with `interp1` and `'pchip'`:
   ```
   example_airfoilAnalytic0515AlPerformance
   ```

For more information, please take a look at the function documentation as well as the literature stated in the function documentation.