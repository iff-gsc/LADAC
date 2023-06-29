# ArduPilot SITL interfaces for Simulink

There are two interfaces available:
The [Adopted Gazebo Interface](#adopted-gazebo-interface) is based on the connection of Gazebo with ArduPilot SITL and uses the same UDP port as well as the same flight parameters.
The [Custom Simulink Interface](#custom-simulink-interface) uses a modification of the ArduPilot code which is provided in LADAC.  
For more information about the ArduPilot SITL interfaces take a look at the 
[ArduPilot Documentation](https://ardupilot.org/dev/docs/sitl-with-gazebo.html), 
the [ArduPilot source code](https://github.com/ArduPilot/ardupilot/blob/master/libraries/SITL/SIM_Gazebo.h),
the [implementation of SwiftGust plugin](https://github.com/SwiftGust/ardupilot_gazebo)
or the [implementation of khancyr plugin](https://github.com/khancyr/ardupilot_gazebo).  
Note that the Gazebo interface does not support all kind of sensors yet while the Custom Simulink Interface provides rangefinder sensors and can easily be extended to include other sensors.


## Motivation

With this interface you can test and tune ArduPilot controllers in simulation given you have a model for your vehicle in MATLAB/Simulink.
There are several other programs that can simulate the flight physics of some vehicles.
However, Simulink has several advantages, especially regarding analysis and insight of the model as well as for control design.


## Installation

- You must install [LADAC](https://github.com/iff-gsc/LADAC#readme) and the [ArduPilot SITL](https://ardupilot.org/dev/docs/SITL-setup-landingpage.html).  
- Optional: install [Mission Planner](https://ardupilot.org/planner/) or [QGroundControl](http://qgroundcontrol.com/).  
- If you want to use the [Custom Simulink Interface](#custom-simulink-interface), you have to patch your local ArduPilot repository according to the instructions in [ArduPilot custom controller](../ArduPilot_custom_controller#readme). 


## Tests

### Adopted Gazebo Interface

With these instructions you can test the connection of Simulink with the ArduPilot SITL.
Only the UDP connection from Simulink to ArduPilot SITL is tested.
This test generates flight parameters for a steady spiral trajectory in a Simulink model.
The parameters of the spiral can be adjusted by a parameter file.

1. Open the MATLAB/Simulink test trajectory simulation
   1. Open MATLAB/Simulink.
   2. Run the parameters file [init_exampleSITL.m](/utilities/interfaces_external_programs/ArduPilot_SITL/init_exampleSITL.m).
   3. Open the Simulink file [example_SITL.slx](/utilities/interfaces_external_programs/ArduPilot_SITL/example_SITL.slx).
   4. Run the Simulink model. The green switch should activate "apply zeros"
        in order to simulate a vehicle standing on the ground.

2. Prepare ArduPilot SITL
   1. Run the SITL **in Gazebo mode** (`--model=gazebo`) from command line specifying the desired parameter, e.g. (you can replace `plane` with `quad` etc.):
      ``````
      sim_vehicle.py -v ArduPlane -f plane --model=gazebo
      ``````
   2. You should arm the vehicle to avoid that the altitude is resetted (same console as above):
      ``````
      arm throttle
      ``````

3. Get visual feedback with ground control station:  
   Run a ground control station software like Mission Planner or QGroundControl
        and establish a connection with the SITL.

4. Activate the test trajectory simulation
   1. In the running Simulink model [example_SITL.slx](/utilities/interfaces_external_programs/ArduPilot_SITL/example_SITL.slx) double-click the green switch
        to activate "apply spiral".
   2. If you want to start a new test trajectory simulation, you have to stop the
        current simulation and repeat the whole procedure.

5. Analyze the results
   1. You can observe how the vehicle state is computed by the ArduPilot SITL
        by the monitoring of the ground control station. No errors should occur
        and the vehicle state should be very similar to the current trajectory
        state (of course consistent data from Simulink is required).
   2. You can plot the results using MATLAB. The trajectory data is saved to
        the Matlab workspace by the Scopes (named "Scope..."). The estimated
        state by the ArduPilot SITL is logged in the /logs folder where you started
        the sim_vehicle.py command. The log can be converted to a .mat file using
        Mission Planner or [ArduPilog](https://github.com/Georacer/ardupilog). Opening the .mat file with Matlab will load several
        data to the Matlab workspace. Comparing the Simulink data with the ArduPilot
        SITL data should only show small deviations. The small deviations occur
        due to slightly manipulated sensor data by the SITL as well as the EKF.


### Adding Debug Output

The following section describes the custom Debug Message to SIMULINK.
    
    
1. Enabling the Interface\
        To activate the basic interface define
           **Custom_Matlab_Output**
        in mode.h and uncomment the 
            **socket_debug.sendto(...)**
        command in mode_custom. ( content of the mode_custom patch )

2. Changes to Simulink\
        To receive the debugging message in Simulink the receive_from_ArduPilot_SITL Block may be used 
        with the following default settings:

            - IP-Address '127.0.0.1'
            - Local Port: 9004
            - Sample time is depending on the SITL sample time
        

3. Customizing the Interface\
        To change the data sent by the Interface you may change the content of the sendto message in mode_custom.  
        The receiving block needs to be changed accordingly.
    
* known issues\
      Due to the additional message expected by SIMULINK and the according Blocking time the simulation slows down significantly until mode_custom is enabled. 
        


### Custom Simulink Interface

Unfortunately, there is no real test example yet.
This is how you start the patched ArduPilot SITL with the custom Simulink interface in terminal:
```
sim_vehicle.py -v ArduPlane --model=simulink
```


## How to use?

There are receive blocks and very similar send blocks in the Simulink library [`ardupilot_sitl_lib`](ardupilot_sitl_lib.slx).
These blocks can be used in your flight dynamics Simulink file.
The send blocks send the vehicle state to ArduPilot SITL (they have different inputs interfaces).
The receive blocks receive the actuator commands from the ArduPilot SITL.
Note that the ArduPilot SITL must be started like in the [Tests](#tests) section (step 2).
