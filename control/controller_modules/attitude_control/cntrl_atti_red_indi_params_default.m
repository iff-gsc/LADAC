% ** INDI reduced attitude controller parameters (default) **

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

%% reference model parameter

% maximum lean value
atti_cntrl.rm.lean.max = deg2rad(45);
% natural frequency of second order lean dynamics, rad/s
atti_cntrl.rm.lean.omega = 11;
% damping ratio of second order lean dynamics, -
atti_cntrl.rm.lean.D = 1;

% maximum yaw rate, rad/s
atti_cntrl.rm.r.max = deg2rad(300);
% time constant of first order yaw rate dynamics, s
atti_cntrl.rm.r.T = 0.1;

%% controller gains

% @[ny_atti_dt2]'/@[e_atti, e_atti_dt, e_atti_dt2]'
% with atti = [roll, pitch, yaw]
atti_cntrl.K = [ ...
   50.0000         0         0   14.7973         0         0    0.1896         0         0; ...
         0   50.0000    0.0000         0   14.7973    0.0000         0    0.1896    0.0000; ...
         0    0.0000   50.0000         0    0.0000   14.8658         0    0.0000    0.2099; ...
    ];
