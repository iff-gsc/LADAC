function params = fuselageParamsInit( n_sections )
%fuselageParamsInit define and initialize fuselage parameters struct
% 
% Inputs:
%   n_sections      number of sections specified in the params_file (scalar)
% 
% Outputs:
%   params          fuselage parameters struct as defined by this function
% 
% See also:
%   fuselageInit, fuselageCreate, fuselageSetGeometry
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% fuselage length, in m
params.total_length = 0;

% longitudinal coordinates of wing parts (1x(s+1) array), dimensionless
params.xi_segments = zeros( 1, n_sections + 1 );
% fuselage width at segment borders (1x(s+1) array), in m
params.width = zeros( 1, n_sections + 1 );
% center line height displacement at segment borders (1x(s+1) array), in m
params.center_line_height = zeros( 1, n_sections + 1 );
% are section between two equal points straight?
params.is_straight = false;

% lift curve slope (normalized by V^(2/3))
params.C_L_alpha = 0;
% zero lift drag coefficient (normalized by V^(2/3))
params.C_D0 = 0;

end

