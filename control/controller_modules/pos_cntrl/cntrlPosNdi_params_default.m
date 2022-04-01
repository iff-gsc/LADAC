% ** NDI position controller parameters (default) **

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

%% reference model parameter

% velocity time constant, in s
psc.rm.veltc  	= 0.8;
% maximum lateral velocity, in m/s
psc.rm.velxymax = 14;
% maximum lateral acceleration, in m/s^2
psc.rm.accxymax = 6.5;
% maximum (absolute) upwards vertical velocity, m/s
psc.rm.velumax 	= 6;
% maximum (absolute) downwards vertical velocity, m/s
psc.rm.veldmax 	= 4;
% maximum (absolute) upwards vertical acceleration, m/s^2
psc.rm.accumax  = 7;
% maximum (absolute) downwards vertical acceleration, m/s^2
psc.rm.accdmax  = 4;

%% position controller gains

% position gain
psc.k.pos 	= 2.1434;
% velocity gain
psc.k.vel 	= 2.8016;
% acceleration gain
psc.k.acc 	= 0.3455;
