# ArduPilot SITL interfaces for Simulink

There are three interfaces available:
The [Adopted Gazebo Interface](#adopted-gazebo-interface) is based on the connection of Gazebo with ArduPilot SITL and uses the same UDP port as well as the same flight parameters.
The [JSON](#json-interface) used the ArduPilot JSON interface but it only works with a modification of the ArduPilot code which is provided in LADAC.
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
   2. Run the parameters file [ardupilot_sitl_example_init.m](/utilities/interfaces_external_programs/ArduPilot_SITL/ardupilot_sitl_example_init.m).
   3. Open the Simulink file [ardupilot_sitl_example.slx](/utilities/interfaces_external_programs/ArduPilot_SITL/ardupilot_sitl_example.slx).
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
   1. In the running Simulink model [ardupilot_sitl_example.slx](/utilities/interfaces_external_programs/ArduPilot_SITL/ardupilot_sitl_example.slx) double-click the green switch
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
        

### JSON Interface

This interface does not work with the original ArduPilot code because the automatic port detection does not work.
That is why this interface must be used with the [modified ArduPilot code](../ArduPilot_custom_controller).
The Simulink blocks `Receive from ArduPilot SITL (JSON)` and `Send to ArduPilot SITL (JSON)` can be found in [ardupilot_sitl_lib.slx](/utilities/interfaces_external_programs/ArduPilot_SITL/ardupilot_sitl_lib.slx).

This is how you start the patched ArduPilot SITL with the JSON interface in terminal:
```
sim_vehicle.py -v ArduPlane --model=JSON
```

With the `Send to ArduPilot SITL (JSON)` block it is possible to send additional data like airspeed and rangefinder distances.
Therefore, put the appropriate JSON string in the Mask parameter "Optional signal string".
For example, if you want to send the airspeed and the first rangefinder distance: `'"airspeed":%f,"rng_1":%f'`
The additional block input "opt_signals" will appear.
For this example you have to connect a signal of length 2 to this inport (first is airspeed, second is rangefinder).
More info: https://github.com/ArduPilot/ardupilot/tree/master/libraries/SITL/examples/JSON  
Notes: 
- If you want to use rangefinders in ArduPilot SITL, you have to set the ArduPilot parameters `RNGFNDx_TYPE=100`.
- If you want to use airspeed in ArduPilot SITL, you have to set the ArduPilot parameters `ARSPD_TYPE=2`, `ARSPD_PIN=1`, and `ARSPD_SKIP_CAL=0`.

### Custom Simulink Interface

Unfortunately, there is no real test example yet.
This is how you start the patched ArduPilot SITL with the custom Simulink interface in terminal:
```
sim_vehicle.py -v ArduPlane --model=simulink
```

### Workaround if no connection can be established between ArduPilot SITL under WSL and MATLAB/Simulink under Windows

If you have set up the ArduPilot SITL in the Windows Subsystem for Linux (WSL2), e.g. Ubuntu 22.04 LTS, and run MATLAB/Simulink under Windows, it may not be possible to establish a connection between them.
The ArduPilot SITL and MATLAB/Simulink communicate via a UDP connection. Unfortunately, WSL2 does not yet fully support UDP port forwarding via localhost (see https://github.com/microsoft/WSL/issues/6082, https://github.com/microsoft/WSL/issues/6351, https://github.com/microsoft/WSL/issues/8610).
But there is a workaround to establish a connection via UDP anyway. With the tool *socat*, UDP packets can be forwarded via a TCP tunnel.  
  
Therefore the following steps need to be executed:  

1. Install *socat* in WSL2 (e.g. Ubuntu 22.04 LTS):  
   ```
   sudo apt install socat
   ```
2. Download *socat* for Windows from https://github.com/tech128/socat-1.7.3.0-windows/archive/refs/heads/master.zip.
3. Extract the zip file to a folder of your choice and add the folder path to the Windows PATH environment variable as described here:  
   [How to: Add Tool Locations to the PATH Environment Variable](https://learn.microsoft.com/en-us/previous-versions/office/developer/sharepoint-2010/ee537574(v=office.14))
4. Execute each of the following commands in a separate window in the WSL (replace `<windows-ip>` with the IP address of your Windows OS, available via `ipconfig`):
   ```
   socat UDP4-LISTEN:9002,fork TCP4:<windows_ip>:9002
   socat TCP4-LISTEN:9003,fork UDP4:localhost:9003
   socat UDP4-LISTEN:14550,fork TCP4:<windows_ip>:14550
   ```
5. Execute each of the following commands in a separate window in PowerShell or CMD under Windows (replace `<wsl-ip>` with the IP address of your WSL distribution, available via `ip a | grep eth0`):
   ```   
   socat TCP4-LISTEN:9002,fork UDP4:localhost:9002
   socat UDP4-LISTEN:9003,fork TCP4:<wsl_ip>:9003
   socat TCP4-LISTEN:14550,fork UDP4:localhost:14550
   ```
6. Now the UDP ports 9002 (ArduPilot SITL to Simulink), 9003 (Simulink to ArduPilot SITL) and 14550 (ArduPilot SITL to GCS, e.g. Mission Planner or QGroundControl) are forwarded.
   It is possible to add more ports (e.g. 5503 for ArduPilot SITL to FlightGear if installed under Windows).
7. Run the Simulink file, start the ArduPilot SITL and open a GCS software if necessary.

* A script is being worked on to simplify UDP port forwarding.  


## How to use?

There are receive blocks and very similar send blocks in the Simulink library [`ardupilot_sitl_lib`](ardupilot_sitl_lib.slx).
These blocks can be used in your flight dynamics Simulink file.
The send blocks send the vehicle state to ArduPilot SITL (they have different inputs interfaces).
The receive blocks receive the actuator commands from the ArduPilot SITL.
Note that the ArduPilot SITL must be started like in the [Tests](#tests) section (step 2).
