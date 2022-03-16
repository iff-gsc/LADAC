%  ** second order actuator parameters (default) **

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% natural frequency, rad/s
act.naturalFrequency = 80;
% damping ratio, 1
act.dampingRatio = 1;
% maximum deflection, rad
act.deflectionMax = deg2rad(25);
% minimum deflection, rad
act.deflectionMin = deg2rad(-25);
% maximum deflection rate, rad/2
act.deflectionRateMax = deg2rad(60/0.18);