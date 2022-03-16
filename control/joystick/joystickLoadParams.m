function jystck = joystickLoadParams( filename, mode, zero_throttle )
% joystickLoadParams loads a struct with joystick parameters from a
%   parameters file.
% 
% Inputs:
%   filename            joystick configuration filename, see params
%                       subfolder (e.g. 'joystick_params_Xbox_360')
%   mode                number that defines the function of the sticks (1,
%                       2, 3 or 4) according to [1]   
%   zero_throttle       define where zero throttle is:
%                       0 means that center stick is zero throttle (e.g.
%                       for loiter modes),
%                       -1 mean that stick down is zero throttle (e.g. for
%                       attitude controller modes).
% 
% Example:
%   jystck = joystickLoadParams( 'joystick_params_Xbox_360', 2, 0 );
% 
% Literature:
%   [1] https://drones.stackexchange.com/questions/186/what-are-modes-of-a-transmitter-controller/187
%   
% See also:
%   joystickCalibrate, joystickCh2Rpyt
%

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% load parameters from file
run(filename);

% channel -> roll,pitch,yaw,throttle
switch mode
    case 1
        jystck.mode_idx = [ 1; 3; 4; 2 ];
    case 2
        jystck.mode_idx = [ 1; 2; 4; 3 ];
    case 3
        jystck.mode_idx = [ 4; 3; 1; 2 ];
    case 4
        jystck.mode_idx = [ 4; 2; 1; 3 ];
    otherwise
        error('The mode was not specified correctly.')
end

switch zero_throttle
    case -1
        jystck.zero_throttle = -1;
    case 0
        jystck.zero_throttle = 0;
    otherwise
        error('Zero throttle was not specified correctly.')
end

jystck.enable = 1;

end