% init parameters of Simulink model

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

clear all

wing = wingCreate('wing_params_default',30,'is_unsteady',true);

struct2bus(wing.state,'wing_state_bus');

open('wing_example_unst_sim')

%% simulate

sim('wing_example_unst_sim','StartTime','0','StopTime','1');