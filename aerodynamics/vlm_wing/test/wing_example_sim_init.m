% init parameters of Simulink model

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

clear all

wing = wingCreate('wing_params_Arkbird_simple',30);

wing_state_bus = struct2bus_(wing.state);

atmosphereBus = struct2bus_(isAtmosphere(0));

open('wing_example_sim')

%% simulate

sim('wing_example_sim','StartTime','0','StopTime','1');