% fuselage parameters (default)

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

%% basic parameters

% fuselage length, in m
total_length = 15;
% lift curve slope
C_L_alpha = 0.3;
% zero lift drag coefficient (normalized by V^(2/3))
C_D0 = 0.05;

%% fuselage sections (s sections)

% longitudinal coordinates of fuselage parts (1x(s+1) array), dimensionless
xi_segments = [ 0, 0.07 0.2 0.65, 0.85, 1 ];
% fuselage width at section borders (1x(s+1) array), in m
border_width = [ 0, 1.5, 2, 2, 1.3, 0.2 ];
% center line height displacement at section borders (1x(s+1) array), in m
center_line_height = [ -0.3, -0.1, 0, 0, 0.05, 0.2 ];
% are section between two equal points straight?
is_straight = true;
