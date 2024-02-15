% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2024 Jonas Withelm
%   Copyright (C) 2024 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% loop time, must be the same as 1/AP:SCHED_LOOP_RATE
param.ts = 0.0025;

% maximum number of waypoints
param.traj.wpmax = 4;
% boolean if last and first waypoint should be connected (true) or not
param.traj.cycle = true;
% degree of spline polynomials
param.traj.degree = 5;
