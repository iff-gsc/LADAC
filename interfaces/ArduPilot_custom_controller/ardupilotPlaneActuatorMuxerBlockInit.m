function ch_fcn = ardupilotPlaneActuatorMuxerBlockInit(num_channels)
% ardupilotPlaneActuatorMuxerBlockInit initialize Simulink block "Actuator
% Muxer ArduPlane"
%   This function is called from the initialization of the Simulink block
%   "Actuator Muxer ArduPlane".
%   It dynamically configures the block mask so that a variable number of
%   inports can be provided. The portnames are adjusted automatically to
%   the specified functions. The function output contains the servo
%   functions as defined by ArduPlane. The servo functions are extended by
%   a "feedthrough" function.
% 
% Syntax:
%   ch_fcn = ardupilotPlaneActuatorMuxerBlockInit(num_channels)
% 
% Inputs:
%   Take a look at the mask of the Simulink block "Actuator Muxer
%                       ArduPlane".
%   num_channels        Number of channels used.
% 
% Outputs:
%   ch_fcn              Array (length of num_channels) of channel functions
%                       as defined by ArduPlane as uint16 (see
%                       ardupilot_plane_servo_functions.txt, 0 corresponds
%                       to the first function, 999 means "feedthrough")
% 
% See also:
%   ardupilotInitLogMuxer, ardupilotCreateInputBuses

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2024 Yannic Beyer
%   Copyright (C) 2024 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

passthrough_str = 'passthrough';
ch_fcn = uint16(zeros(16,1));
channel_type_options = channelTypeOpts();
channel_type_options = [ {passthrough_str}; channel_type_options ] ;
% channel_type_options = [ channel_type_options; {'passthrough'} ] ;
maskObj = Simulink.Mask.get(gcb);
port_display_str = [ ...
    "port_label('output',1,'channels');"; ...
    "port_label('output',2,'function_channels');" ...
    ];
for i = 1:16
    block_name = ['Channel_',num2str(i)];
    is_enabled = i <= num_channels;
    blockInitToggleInport(block_name,is_enabled);
    maskObj.Parameters(i+1).TypeOptions = channel_type_options;
    if strcmp(maskObj.Parameters(i+1).Value,passthrough_str)
        ch_fcn(i) = 999;
    else
        ch_fcn(i) = find(strcmp(maskObj.Parameters(i+1).Value,channel_type_options))-2;
    end
    if is_enabled
        maskObj.Parameters(i+1).Visible = 'on';
        if strcmp(maskObj.Parameters(i+1).Value,passthrough_str)
            port_display_str_i = join(["port_label('input',",num2str(i),",'Channel ",num2str(i)," (passthrough)');"],"");
        else
            port_display_str_i = join(["port_label('input',",num2str(i),",get_param(gcb,'CHFCN",num2str(i),"'));"],"");
        end
        port_display_str = [ port_display_str; port_display_str_i ];
    else
        maskObj.Parameters(i+1).Visible = 'off';
    end
end

maskObj.Display = char(port_display_str');

end

function channel_type_options = channelTypeOpts()

fid = fopen('ardupilot_plane_servo_functions.txt');
servo_fnc_cell = textscan(fid,'%s');
channel_type_options = servo_fnc_cell{:};

end