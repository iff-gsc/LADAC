% Example for fuselage Simulink library blocks

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% init
fuselage = fuselageCreate('fuselage_params_default',5,20);
struct2bus(fuselage.state,'fuselage_state_bus');
atmosphere_struct = isAtmosphere(0);

% this part is only needed if this script is called from a function
% (to do: improve struct2bus function so that Simulink bus objects are
% created in caller workspace)
assignin('base','fuselage',fuselage)

% run Simulink model
options = simset('srcWorkspace','current');
sim('fuselage_lib_example')
