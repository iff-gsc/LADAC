% Flexible-body equations of motion: Initialization of the Simulink example

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2025 Yannic Beyer
%   Copyright (C) 2020-2025 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% Create default structural dynamics beam-element model
structure = structureCreate( 'structure_params_default' );

% Create modal structural dynamics model
num_modes = 10;
structure_red = structureGetReduced(structure,num_modes);

open('flexibleBody_lib_example');