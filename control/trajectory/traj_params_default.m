% ** trajectory parameters (default) **

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% maximum number of waypoints
param.wpmax = 4;

% boolean if last and first waypoint should be connected (true) or not
param.cycle = true;

% degree of spline polynomials
param.degree = 5;