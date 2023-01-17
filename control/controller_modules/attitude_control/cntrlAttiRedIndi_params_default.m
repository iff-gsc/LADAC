% ** INDI reduced attitude controller parameters (default) **

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

%% reference model parameter

% maximum lean value
param.rm.leanmax	= deg2rad(45);
% natural frequency of second order lean dynamics, rad/s
param.rm.leanfreq	= 11;
% damping ratio of second order lean dynamics, -
param.rm.leandamp	= 1;

% maximum yaw rate, rad/s
param.rm.yawratemax	= deg2rad(300);
% time constant of first order yaw rate dynamics, s
param.rm.yawratetc	= 0.1;

%% controller gains

% lean angle error gain, 1/s^2
param.k.lean        = 50;
% lean rate error gain, 1/s
param.k.leanrate	= 14.7973;
% lean acceleration error gain, -
param.k.leanacc     = 0.1896;

% yaw angle error gain, 1/s^2
param.k.yaw         = 50;
% yaw rate error gain, 1/s
param.k.yawrate     = 14.8658;
% yaw acceleration error gain, -
param.k.yawacc      = 0.2099;
