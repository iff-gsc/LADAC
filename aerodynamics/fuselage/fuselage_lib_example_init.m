% Example for fuselage Simulink library blocks

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% init
fuselage = fuselageCreate('fuselage_params_default',5,20);
fuselage_state_bus = struct2bus_(fuselage.state);
atmosphere_struct = isAtmosphere(0);
atmosphereBus = struct2bus_(atmosphere_struct);

% this part is only needed if this script is called from a function
% (to do: improve struct2bus function so that Simulink bus objects are
% created in caller workspace)
assignin('base','atmosphereBus',atmosphereBus)
assignin('base','fuselage',fuselage)
assignin('base','fuselage_state_bus',fuselage_state_bus)

% run Simulink model
options = simset('srcWorkspace','current');
sim('fuselage_lib_example')
