% Simple airfoil parameters (default)

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% c_L = c_L_alpha * ( alpha - alpha_0 )
% c_D = c_D0 + c_D1*alpha + c_D2*alpha^2
% c_m = c_m0 + x_ac * c_L

% lift curve slope, in 1/rad
c_L_alpha 	= 2*pi;
% zero lift angle of attack, in rad
alpha_0 	= 0;

% drag coefficient at zero angle of attack, dimensionless
c_D0    	= 0.03;
% c_D1, in 1/rad
c_D1        = 0;
% c_D2, in 1/rad^2
c_D2     	= 0.75;

% pitching moment coefficient at zero lift with respect to the 25% chord
% position, dimensionless
c_m0        = 0;
% aerodynamic center (ac) position relative to the chord (backward positive,
% 0.25 means that ac is at 25% chord position), dimensionless
x_ac        = 0.25;

