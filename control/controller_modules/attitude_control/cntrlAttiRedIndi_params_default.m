% ** INDI reduced attitude controller parameters (default) **

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

%% reference model parameter

% maximum lean value
atc.rm.leanmax      = deg2rad(45);
% natural frequency of second order lean dynamics, rad/s
atc.rm.leanfreq     = 11;
% damping ratio of second order lean dynamics, -
atc.rm.leandamp     = 1;

% maximum yaw rate, rad/s
atc.rm.yawratemax   = deg2rad(300);
% time constant of first order yaw rate dynamics, s
atc.rm.yawratetc    = 0.1;

%% controller gains

% lean angle error gain, 1/s^2
atc.k.lean      = 50;
% lean rate error gain, 1/s
atc.k.leanrate  = 14.7973;
% lean acceleration error gain, -
atc.k.leanacc   = 0.1896;

% yaw angle error gain, 1/s^2
atc.k.yaw       = 50;
% yaw rate error gain, 1/s
atc.k.yawrate   = 14.8658;
% yaw acceleration error gain, -
atc.k.yawacc    = 0.2099;
