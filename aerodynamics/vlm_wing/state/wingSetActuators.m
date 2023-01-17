function wing = wingSetActuators( wing, actuators_pos, actuators_rate )
%setActuator sets the deflection of the flaps
%   Each panel can be controlled differently according to the control 
%   input index. The function setActuators determines the deflection of the 
%   flaps regarding the actuator states and the control input index. 
%
% Inputs:
% 	 geometry               Wing geometry
%                           (struct, see: wingSetGeometry)
%    actuatorState          Current actuator states
%                           (array)
%    actuatorIndex          Control index of each section
%                           (array)
%
% Outputs:
%    deflection             Deflection of each panel considering the type 
%                           of flap and its actuator state 
%                           (array) 

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2021 Lucas Schreer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% actuator states
wing.state.actuators.pos(1:length(actuators_pos)) = actuators_pos;
wing.state.actuators.rate(1:length(actuators_rate)) = actuators_rate;

%% set deflection according to control input index and actuator state
% add zero for segments without actuator
actuators_pos = [ 0, wing.state.actuators.pos ];
actuators_rate = [ 0, wing.state.actuators.rate ];
wing.state.actuators.segments.pos(:) = actuators_pos(wing.geometry.segments.control_input_index_local(:)+1);
wing.state.actuators.segments.rate(:) = actuators_rate(wing.geometry.segments.control_input_index_local(:)+1);

end