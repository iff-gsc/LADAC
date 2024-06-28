function [] = wingSetCustomActuatorPath(wing)
% wingSetCustomActuatorPath sets the Matlab path to select custom actuator
%   The wing project allows the use of custom actuators, for which the 
%   corresponding function must be specified. However, as Simulink does not
%   allow handle functions, it is not possible to specify the name of the
%   function. 
%   The following solution is used:
%   The function for the custom actuator must have the name
%   "wingCustomActuator", whereby the function "wingCustomActuatorSetup"
%   is called during initialization. Both functions must be stored in a
%   folder with a unique name. The name of the folder is then specified and
%   this function (wingSetCustomActuatorPath) adds the specified folder to
%   the Matlab path and removes all other folders that also contain a
% 	function with the name wingCustomActuator from the path.
% 
% Syntax:
%   wingSetCustomActuatorPath(wing)
% 
% Inputs:
%   wing          	wing struct (see wingCreate)
% 
% See also:
%   wingCreate, wingCustomActuator, wingCustomActuatorSetup

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2023-2024 Yannic Beyer
%   Copyright (C) 2023-2024 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

custom_path = which('wingCustomActuator','-all');
if length(custom_path) > 1
    custom_act_split = strsplit(wing.params.actuator_2_type(1,:),'-');
    custom_act_name = custom_act_split{end};
    for i = 1:length(custom_path)
        folder_names_split = strsplit(custom_path{i},{'/','\'});
        custom_folder_name = folder_names_split{end-1};
        if ~strcmp(custom_folder_name,custom_act_name)
            rmpath(fileparts(custom_path{i}));
        end
    end
end
custom_path = which('wingCustomActuator','-all');
if length(custom_path) > 1
    error('Custom actuator was not specified correctly.');
end

end
