function copter = copterLoadParams( filename, varargin )
% copterLoadParams loads a quadcopter parameters struct.
% 
% Example:
%  copter = copterLoadParams( 'copter_params_default' );
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2023 Jonas Withelm
%   Copyright (C) 2023 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************



%% Input Parsing
p = inputParser;
addParameter(p, 'Battery', 'default', @(x) isstring(x)||ischar(x) );
parse(p, varargin{:});

% outputs
battery = p.Results.Battery;


run(filename);


% Component: Battery
if ~isfield(copter, 'bat')
    if strcmp(battery, 'default')
        error('UAV parameter file does not contain a battery configuration. You must specify a battery configuration!');
    end
    
    % Load battery
    if ~exist(battery, 'file')
        error('No battery file for ''%s'' found!', battery);
    end
    run(battery);
    copter.bat = bat;
    
    % Load battery position
    bat_pos_file = ['bat_pos_params_' copter.name];
    if ~exist(bat_pos_file, 'file')
        error('No battery position file for ''%s'' found!', copter.name);
    end
    run(bat_pos_file);
    idx = find(ismember({bat_pos.name}, battery));
    if isempty(idx)
        error('No battery position for battery ''%s'' found!', battery);
    end
    copter.bat.config.CoG_Pos_c = bat_pos(idx).bat_pos_c;
    
    copter = aircraftAddBody(copter, copter.bat);
end


% Initialize propeller map from propeller name
if isfield(copter.prop, 'correction_factor')
    copter.prop.map_fit = propMapFitCreate( copter.prop.name, copter.prop.correction_factor );
else
    copter.prop.map_fit = propMapFitCreate( copter.prop.name, [copter.prop.thrust_factor, copter.prop.torque_factor] );
end

end
