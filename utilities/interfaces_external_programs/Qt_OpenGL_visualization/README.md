# Visualization of aircraft dynamics with Qt OpenGL

Since Matlab/Simulink has limited capabilities in 3D visualization, you can
use this external C++ program to visualize your aircraft.
The program is based on Qt 5.12.8.

## Test

Initialize the example dummy Simulink file:
    ```
    init_struct
    ```

Run the exammple dummy Simulink file:
    ```
    sim('udp_sender_18b')
    ```

Build and run the Qt project.