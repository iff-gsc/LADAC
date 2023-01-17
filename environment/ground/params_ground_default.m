% ** Default parameters for ground model **

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% Maximum acceleration, m/s^2
grnd.acc_max = 300;
% Ground softness parameter (>0), m. Close to zero means solid ground.
grnd.softness_parameter = 1e-02;
% Ground spring stiffness, N/m
grnd.spring_stiffness = 1000;
% Ground damping, N.s/m
grnd.damping = 100;
% Ground friction coefficient, -
grnd.friction_coef = 0.1;
% Faktor for friction reduction (>0) at low speed, -. Large value for 
% realistic results, small value to avoid shattering and slow simulation.
grnd.low_speed_friction_reduction_factor = 10;