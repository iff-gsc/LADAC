% ** Parameters for tunability post-processing of code export (default) **

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2024 Jonas Withelm
%   Copyright (C) 2024 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************



%% Load dependent settings
param = loadParams( 'apPar_params_dependent' );



%% User settings
param.target = 'ArduCopter';        % Define the target (ArduCopter or ArduPlane)

param.delimiter         = '_';      % Delimiter used for parameter names in ArduPilot (ToDo: Allow empty, empty will lead to errors right now)
param.array_delimiter   = '';       % Delimiter between parameter name and array index in ArduPilot

param.use_variable_name = false;    % Also use the variable name for the parameter name in ArduPilot

param.debug = false;                % Print debug messages during processing
