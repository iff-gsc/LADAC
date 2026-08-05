# Tunable MATLAB/Simulink Controller in ArduPilot (beta)
This toolchain extends [ArduPilot custom controller](../) and allows **tunable controller implementations in ArduPilot**. If you have not already done so, please learn how to use [ArduPilot custom controller](../README.md) first!

**Tunable** means, that the parameters used to parametrize the Simulink controller model are integrated into the ArduPilot parameter ecosystem and can be changed through any Ground Control Software (see also [Restrictions](#Restrictions)). This eliminates the need to recompile and reflash the custom ArduPilot firmware when the custom controller parameterization has changed, resulting in a huge acceleration of the flight test workflow. However, it must be said that this implementation is hacky and must be considered **early beta**, meaning that it may result in non-compilable, or worse, compilable but crashing code.



## VERY IMPORTANT NOTES FOR FLYING
- Read this README and the [Restrictions](#restrictions).
- This is a beta solution that can lead to crashes. Therefore you should check that everything works in the SITL before flying.
- Before flashing an flightcontroller, make a full backup of the ArduPilot parameters.
- After flashing, verify that the parameters of ArduPilot and the custom controller are still correct (using the backup file and the generated custom controller parameter file). If the total number or the arrangement of the parameters have changed, it is very likely that updating the flightcontroller leads to wrong parameter values (see also [Restrictions](#restrictions)).



## How to use?

### 1. Preparation of a Simulink model for tunable code generation
The Simulink model configuration must be changed so that the code generation produces code with tunable (and not inlined) parameters. These settings can be found in *Model Configuration Parameters -> Code Generation -> Optimization -> Default parameter behavior*. There is a global setting that changes the behavior for all model parameters and a hidden configuration that allows individual settings per parameter (it opens with a click on *Configure...*). For our purposes we use the latter one.

We provide a MATLAB function for automatic model configuration (see also [`apParAddTunability2Model()`](./apParAddTunability2Model.m)) so that no manual model changes are required. However, it is still useful to know which model settings are changed by the function and where they can be found. In addition to the parameter tunability settings, the function also changes the model settings in *Model Configuration Parameters -> Custom Code -> Source file / Header file*, which is needed for post-processing the generated code.

To configure a model for tunability, make sure to start with a clean model (no unsaved changes, code generation working) and do:

1. Open and initialise the Simulink model
2. Run `apParAddTunability2Model(<model-name>)`. Please consider the [Restrictions](#restrictions) if an error occurs.
3. Try to generate code for the model (*Build Model*)

If the code generation fails (which is very likely), the model must be adapted to be compatible with tunable code generation. The changes required depend heavily on the model in question, but there are some general points to bear in mind:

- **Sample time from variable:**

    If the model gets its sample time (in the model solver settings or at input / output ports) from a variable, this variable must be non-tunable (inlined). Simulink does not support a tunable sample time for the fixed-step solver. If all of the models parameters are in one struct, this struct should be divided into two structs where the one containing the non-tunable (inlined) parameters should be suffixed with `_notune` ([`apParAddTunability2Model()`](./apParAddTunability2Model.m) takes the `_notune` keyword into account). Once these changes have been made, simply start again at point 2.
    
    Example: If the parameters struct is named `lindi`, create two structs from it, one still named `lindi` (containing the tunable parameters) and the other one named `lindi_notune`(containing the non-tunable parameters).

- **Expression limitations:**

    Consider the [Limitations for Block Parameter Tunability in Generated Code](https://mathworks.com/help/rtw/ug/limitations-for-block-parameter-tunability-in-the-generated-code.html).

- **Expressions in masks:**

    If the model uses blocks with masks, [loss of tunability](https://mathworks.com/help/simulink/gui/detect-loss-of-tunability.html) errors can occur if expressions (e.g. `a ./ b`) using variables of data type `single` are used in the masks. Please have a look at [Mask Expression Handling](https://github.com/iff-gsc/m-Utils/tree/main/simulink/mask_expression_handling#readme) for a solution. If the blocks have *Mask Expression Handling* already implemented, make sure it is switched on.

If the code generation still does not work after taking all the above points into account, it is possible to exclude problematic parameters from tunability by moving them into the non-tunable (inlined) parameters struct with the `_notune` suffix.

### 2. Post-Processing of the generated code

1. Open and initialise the Simulink model
2. Generate code for the model (*Build Model*)
2. Run `apParProcessCodeExport( 'apPar_<copter/plane>_params_default' )`. Please consider the [Restrictions](#restrictions) if an error occurs.

This will generate the following additional files in the code export directory of the model:

- `MatlabControllerStructOverride.h`
- `MatlabControllerParams.cpp`
- `QGC_Params.params`

The C++ files must also be copied to the target directory for the custom controller in ArduPilot (see [ArduPilot custom controller](../README.md#how-to-use)). The `QGC_Params.params` file is a parameter file for QGroundControl that contains all tunable model parameters and their values.

After copying the custom controller to ArduPilot you can try to compile ArduPilot and test everything in the SITL. Use QGroundControl to verify that the custom controller parameters are visible and can be changed. The parameters of the custom controller are preceded by the prefix “ML_”.

If ArduPilot cannot be compiled, please take a look at the [Technical Background](#technical-background) and [Restrictions](#restrictions). If ArduPilot compiles but crashes, please make sure, that the parameters have their correct values.

### 3. Generate updated parameter file
If the model itself is unchanged and only the parameter calculation has changed, you can do:

1. Open and initialise the Simulink model
2. Run `apParGenerateQGCParFile( 'apPar_<copter/plane>_params_default' )`

to generate a new parameter file for QGroundControl. The file is saved again in the code export directory of the model and the file name is given a time stamp as a prefix, e.g. `20240531_10-42-53_QGC_Params.params`.



## Technical background
The ArduPilot parameter ecosystem is based on the [AP_Param](https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Param) class. The basic scalar data types are realized through template instantiations. The following table lists all scalar data types that are used in ArduPilot:

| ArduPilot | MATLAB | Simulink |
| --- | --- | --- |
| AP_Float | single | real32_T |
| AP_Int8 | int8 | int8_T |
| AP_Int16 | int16 | int16_T |
| AP_Int32 | int32 | int32_T |

In a comment in [AP_Param](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Param/AP_Param.h#L883) it is stated, that
> Objects of this type have a value, and can be treated in many ways as though they were the value.

The post-processing scripts for the generated code ([`apParProcessCodeExport()`](./apParProcessCodeExport.m)) therefore simply replace the C++ data types of the tunable parameters with the matching ArduPilot data types. This is done through a rudimentary parsing of the generated code and the generation of `MatlabControllerStructOverride.h`, which gets then included in `MatlabController.h`. In this way the above-mentioned scalar types and 1D arrays of these scalars can be used as model parameters.

This approach works in most cases but is a "hacky" solution, because it replaces the fundamental data types in the data structures of the generated code. Especially with regard to array handling, this can lead to problems and uncompilable code that has to be corrected manually.

The default values for the model parameters are defined in the other file (`MatlabControllerParams.cpp`).



## Restrictions

#### Supported data types
Right now, only scalar structs are supported. The structs can contain nested fields. The fields data types must be scalars or 1D arrays of the supported types (see [Technical Background](#technical-background)).

#### Arrays
Only 1D arrays can be used as model parameters.

#### Parameter names
The nested structs fieldnames are used to generate the parameter names in the ArduPilot ecosystem. All parameters are prefixed with "ML_". If an scalar parameter is e.g. `lindi.atc.k.lean` it is translated to `ML_atc_k_lean`. An array parameter would be translated from `lindi.cep.nx(1)` to `ML_cep_nx1`. ArduPilot has a character limit of 16 characters for parameter names. If the resulting parameter name would be too long, it will be shortened automatically, but the shortening may fail if the name is too long. It is therefore best to keep the characters limit in mind when adding new parameters. The name translation behaviour can be customized in `apPar_<copter/plane>_params_default.m`
Furthermore, ArduPilot uses only upper case letters for the parameter names. The use of upper and lower case letters offers more possibilities for better naming schemes, but is not supported by every ground control station.

#### Supported Ground Control Stations
- **MAVProxy**

    Does not support parameter names that contain lower case letters and can therefore not be used to configure parameters of the custom controller.

- **QGroundControl**

    Supports lower case letters in parameter names and should be used to configure parameters of the custom controller.

#### Parameter value range
The ground control stations do not have any information about the parameters of the custom controller, meaning they can not know if the input value is within a valid range. Setting a parameter to zero should be well thought, because it can lead to a divide by zero floating point exception and cause ArduPilot to crash!

#### Parameter persistence
If the parameter struct changes (e.g. the total number of nested parameters, the arrangement of the parameters, ...) it is most likely, that after an software update the parameter values have changed and are wrong! The parameters must be backuped before an update and checked afterwards!

#### Total number of parameters
The maximum number of tunable parameters a model can have is high but not unlimited. ArduPilot has a restriction of 64 parameters per group and has a nesting limit of groups. This toolchain automatically splits the tunable parameters into several groups, but the splitting may fail if there are too many parameters.
