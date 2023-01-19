## Joystick to control Simulink models

This project provides a calibrated joystick that ensures comparable outputs for different controllers and different operating systems.

## Motivation

There are joystick blocks available in Simulink in the "Aerospace Blockset" and "Simulink 3D Animation".
However, if different joysticks on different operating systems are used, the outputs of these blocks will not be identical.
It can happen that the number of outputs is different, the order of channels is different, the trim and maximum values are different and so on.
In addition, not every user of LADAC has a license for the "Aerospace Blockset" or "Simulink 3D Animation" toolbox.
Therefore this projects uses a "free" joystick implementation called "HebiJoystick Input" which does not depend on Simulink toolboxes and was further developed from "MatlabInput" (See also: https://github.com/HebiRobotics/MatlabInput and many thanks to the authors for creating this library!).
To address the problem of using different joysticks on different operating systems a wrapping block with the name 'calibrated joystick' was created and introduces a joystick configuration file.
The configuration file can be generated automatically with a calibration function.

## Installation

- You must install [LADAC](../../../README.md).
- You need to connect a joystick to your computer that is recognized by your operating system.


## Tests

1. Initialize a joystick struct with predefined configuration.
	```
	jystck = joystickLoadParams( 'joystick_params_Xbox_360', 2, 0 );
	```
2. Open the Simulink model `joystick_example`.
	```
	open('joystick_example')
	```
3. Run the Simulink model and double-click the Scope.
4. Move all sticks in all directions and compare the signals (check the legend) with your stick inputs.  
Note that currently only 8 Channels can be used (usually that should be enough).  
Your joystick will most probably not be configured correctly.  
If you think that your stick is not configured correctly, take a look at the [How to use?](#How) section.
5. Copy&paste or drag&drop the Simulink block "Calibrated Joystick" of the (joystick)[joystick.mdl] library
(the library also appears in the Simulink Library Browser if you have installed LADAC correctly).

## How to use?

- Read through the (Tests)[#Tests] section.
- If the joystick does not work as intended: read the documentation of the function
`joystickLoadParams` and check whether you just have to change the `mode` or the `zero_throttle` parameter.
	```
	help joystickLoadParams
	```
- Check if there are already other calibration files that work for your joystick in the [params](params) subfolder.  
If so, repeat the steps of the [Tests](#Tests) section but replace `joystick_params_Xbox_360` with a different filename.
- If you need to calibrate your joystick, read the documentation of the function `joystickCalibrate` and then run the calibration.
	```
	help joystickCalibrate
	joystickCalibrate( 'myJoystick' )
	```
- Check if you got a good calibration file by repeating the steps of the [Tests](#Tests) section
but replace `joystick_params_Xbox_360` with your configuration file `joystick_params_myJoystick`.
- Normally, the test should work like clockwork.  
If something is not as intended, you should repeat the calibration (the old calibration file will be overwritten).
- If there is still a problem, you can study the calibration file `joystick_params_myJoystick`.  
You could also run the Simulink file `joystick_setup` and find out the correct calibration settings manually.
