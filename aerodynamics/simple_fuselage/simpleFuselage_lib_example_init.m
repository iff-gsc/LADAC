% Example for simple fuselage Simulink library block

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% init
% fuselage volume, in m^3
V = 10;
simple_fuselage = simpleFuselageCreate('simpleFuselage_params_default',V);

% run Simulink model
options = simset('srcWorkspace','current');
sim('simpleFuselage_lib_example',[],options)
