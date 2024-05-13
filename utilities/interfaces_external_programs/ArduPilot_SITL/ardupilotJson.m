function [y,y_uint8]  = ardupilotJson(time,Omega_Kb,accel_b,s_g,q_bg,V_Kg,var_str,var_f)
% ardupilotJson create a string to send flight data to the ArduPilot JSON
% interface
% 
% Syntax:
%   y  = ardupilotJson(time,Omega_Kb,accel_b,s_g,q_bg,V_Kg)
%   y  = ardupilotJson(time,Omega_Kb,accel_b,s_g,q_bg,V_Kg,var_str,var_f)
% 
% Inputs:
%   time                    Simulation time (scalar), in s
%   Omega_Kb                Angular rates measured by IMU in body frame
%                           (3x1 array), in rad/s
%   accel_b                 Acceleration measured by IMU in body frame (3x1
%                           array), in m/s/s
%   s_g                     Position in NED frame (3x1 array), in m
%   V_Kg                    Velocity in NED frame (3x1 array), in m/s
%   var_str                 Optional JSON signals string (1x? char)
%                           With this parameter optional input signals to
%                           the Simulink block can be specified. For
%                           example, to specify airspeed and the rirst
%                           rangefinder distance, use:
%                           '"airspeed":%f,"rng_1":%f'
%                           More info can be found here: [1]
%   var_f                   Optional signals as specified by the input
%                           var_str (1xN array, where N is the number of
%                           "%f" occurrences in the input var_str
% 
% Outputs:
%   y                       ArduPilot JSON string with flight data (1x? 
%                           char)
%   y_uint8                 Output y converted to uint8 (1x? uint8)
% 
% Examples:
%   y = ardupilotJson(1,zeros(3,1),zeros(3,1),zeros(3,1),[1;0;0;0],zeros(3,1))
%   y = ardupilotJson(1,zeros(3,1),zeros(3,1),zeros(3,1),[1;0;0;0],zeros(3,1),'"rng_1":%f,"airspeed":%f',[10;20])
% 
% Literature:
%   [1] https://github.com/ArduPilot/ardupilot/tree/master/libraries/SITL/examples/JSON
% 
% See also:
%   ardupilotJsonBlockInit

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2024 Yannic Beyer
% *************************************************************************

if nargin == 6 
    var_str = [];
    var_f = [];
end
var_f_cell = cell(1,length(var_f));
for i = 1:length(var_f_cell)
    var_f_cell{i} = var_f(i);
end

y = sprintf(['\n{"timestamp":%f,"imu":{"gyro":[%f,%f,%f],"accel_body":[%f,%f,%f]},"position":[%f,%f,%f],"quaternion":[%f,%f,%f,%f],"velocity":[%f,%f,%f]',var_str,'}\n'],time,Omega_Kb(1),Omega_Kb(2),Omega_Kb(3),accel_b(1),accel_b(2),accel_b(3),s_g(1),s_g(2),s_g(3),q_bg(1),q_bg(2),q_bg(3),q_bg(4),V_Kg(1),V_Kg(2),V_Kg(3),var_f_cell{:});
y_uint8 = uint8(y);

end