% ** Default parameters for dynamic battery model (not finished) **

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% % parameter obtained by
% load('Elektromodellflug')
% [ SoC_full, SoC_nom, SoC_exp, V_full, V_exp, V_nom, C_rate ] = ...
%   batteryAverageParams(Elektromodellflug);

% % the resistance is determined by curve fitting
Q = SoC_full * obj.params.C/3600;
% the parameters have been determined using the MATLABS intern
% curve fitting tool 
p.R = 0.1077 / (Q*(0.1555*Q+0.9825*obj.params.C_rate_max)^0.5485);
%     p.R = 0.0158;
p.R_over_C = p.R / ( obj.params.C / 3600 );

% [Eo, A, K] = batteryDischargeParams( [SoC_full, SoC_nom, SoC_exp, ...
%   V_full, V_exp, V_nom, C_rate, R_over_C)

SoC_full = points(1);
SoC_nom = points(2);
SoC_exp = points(3);
V_full = points(4);
V_exp = points(5);
V_nom = points(6);
C_rate = points(7);
R_over_C = points(8);

Eo = params.Eo;
params.SoC_full = 1.0618;
R_over_C = params.R_over_C;
K = params.K;
A = params.A;
B = params.B;
