# Pertubation Linear ANalysis Toolbox (PLAN)


## Motivation

Linear approximations of nonlinear systems are used
for system analysis and control design. The Plan toolbox
allows to linearize Simulink models online during the
runtime.


## How to use?

1. Prepare the plant and controller model for linearization
    
    - Save parts of your closed-loop simulation in separate Simulink models. It is
    required for model changes. After using the PLAN toolchain it is difficult and
    challenging to maintain the simulation plant model and the linearized model. The
    following models schould be saved:
        - plant model
        - controller

2. Create a measurement for the Simulink model

    - Create an integrator mesurement bus for the Simulink model which
    is used for online linearization. Use the following example code to create a
    measurement bus:
        ```
        busHandle = createIntegratorOutBus( 'MySimulation/MyPlant Model', 'MyBusName', 'Integrator', 1 );
        busInfo = createBusObjMFile( busHandle );
        ```
    - You should now have a bus selector at your Simulink model top level, an m-function
    in your current folder to create the bus definition and a bus in the workspace.

3. Replace the integrators of the Simulink model
    
    - Reopen the original Simulink model an run the linerator function:
        ```
        linerator( 'MySimulation/MyPlant Model', 'MyBusName', 'Integrator', 1 );
        ```
    - You should now have additional inputs and outputs at your Simulink model top level.
    The inputs are connected with the outputs of the integrator blocks, the outputs
    are connect with the inputs of the integrator blocks. The integrator blocks
    were deleted.

4. Connect Simulink models from step 2 and 3

    - Copy and paste the model from step 3 into the model from step 2.

5. Insert the pertubator in the Simulink model

    - Insert the block pertubator from the LADAC library control/plan.
    - Define the bus definitions in the mask of the block (e.g. for the statesPertub
    bus you want to use the bus definition from step 2).
    - Now, the inputs of the pertubator have to be connected with the model from
    step 2 while the outputs have to be connected with the model from step 3.
    - Connect the input statesPertub and statesNotPertub with the 
    bus created in step 2. The inputs statesNotPertub can also contain
    additional signals. 
    - Connect the input controlInputs to a bus signal
    containing the control inputs from the model from step 2.
    - Connect the output controlInputsPertub_out with the control input from
    step 3. 
    - Connect the output statesPertub_out with the inputs generated
    in step 3 (therefore, you need a bus selector). 
    - Optional: If you added
    signals to the statesNotPertub input, you have to connect them to the
    respective port of the model from step 3 (else you can connect it to a
    terminator). 
    - The outputs pertubDelta and i will be connected later.

6. Insert the analyzer in the Simulink model and obtain state-space representation
    - Insert the block analyzer from the LADAC library control/plan.
    - Connect the outputs generated in step 3 with the input states_dtPertub
    (therefore, you have to use a muxer to merge the signals).
    - Optional: Connect the control inputs signal of the model from step 3 to the 
    input controlInputsPertub. 
    - Optional: Connect an output signal of the model
    from step 3 to the input outputsPertub. 
    - Connect the outputs of the pertubator
    block (step 5) pertubDelta and i with the corresponding inputs of the analyzer.
    - The outputs of the analyzer are the matrices of the linear state-space
    representation.

7. (optional) Modify the obtained state-space representation
    - You can simplify the obtained matrices conducting a reduction of the
system order using the function modredEmb. 
    - Of course, other user defined 
modifications are possible.

8. (optional) Replace other time-dependent blocks
    - The state-space representation (step 6) only is correct if there remain
    no time-dependent blocks in the pertubated model (step 3). In the 
    Simulink library browser all blocks in the sublibraries Continuous and
    Discrete are time-dependent. Moreover, most blocks in the sublibrary Sources
    and some blocks in the sublibrary Discontinuities are time-dependent. 
    - In this toolbox, there are replacers for rate limiters (rateLimiterReplacerWOSat,
    rateLimiterReplace). These replacers approximate the original blocks with
    blocks including an integrator (additional state). This integrator will then
    be replaced in step 3.

9. (optional) Replace discontinuities

    - Linearization of discontinuities is difficult or undefined. That is why
    you may want to replace discontinuities by continuous approximations of
    the discontinuities. 
    - In this toolbox, there is a replacer for saturations
    (saturationReplacer and saturate). 
