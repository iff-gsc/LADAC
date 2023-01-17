function [Eo, A, K, B] = batteryParamsFromCurve( DoD_exp, ...
    DoD_nom, DoD_max, V_full, V_exp, V_nom, C_rate, R_times_C )
% batteryParamsFromCurve computes the parameters of the discharge curve
%   based on three characteristic points of the discharge curve
%   
%   This function computes the parameters of the discharge curve according
%   to the model of Trembley (2009) [TO DO].
%
% Syntax:
%   [Eo, A, K, B] = batteryParamsFromCurve( DoD_exp,DoD_nom,
%       V_full, V_exp, V_nom, C_rate, R_times_C_times_C_rate )
%
% Inputs:
%   DoD_exp     state of charge at exp point (scalar), -
%   DoD_nom     state of charge at nom point (scalar), -
%   DoD_max     depth of discharge at maximum discharge (scalar) (This
%               parameter should obviously be 1, but it has been shown that
%               the model gives better results when this parameter is
%               slightly larger than 1. Indeed, this parameter indicates
%               the point of discontinuity of the model. This discontinuity
%               should rather be at 1.1 or 1.2.)
%   V_full      cell voltage of full battery at given C rate (scalar), V
%   V_exp       cell voltage at exp point at given C rate (scalar), V
%   V_nom       cell voltage at nom point at given C rate (scalar), V
%   R_times_C   product of cell internal resistance and capacity (scalar),
%               Ohm*As
%
% Outputs:
%   Eo          parameter Tremblay's battery model (scalar)
%   A           parameter Tremblay's battery model (scalar)
%   K           parameter Tremblay's battery model (scalar)
%   B           parameter Tremblay's battery model (scalar)
% 
% Literature:
%   [1] Tremblay, O., & Dessaint, L. A. (2009). Experimental validation of
%       a battery dynamic model for EV applications. World electric vehicle
%       journal, 3(2), 289-298.
%
% See also:
%   batteryAverageLiPoCurve

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% equation to calculate the 3 parameters Eo, A, K of the discharge curve
% according to Trembley
SoC_full = DoD_max;
C_rate_SI = C_rate / 3600;
B = 3/DoD_exp;
M_A = [1, 1, -SoC_full/(SoC_full-0)*(0+C_rate_SI) ; ...
    1, exp(-B*DoD_exp), -SoC_full/(SoC_full-DoD_exp)*(DoD_exp+C_rate_SI) ; ...
    1, exp(-B*DoD_nom), -SoC_full/(SoC_full-DoD_nom)*(DoD_nom + C_rate_SI)];
b = [V_full + R_times_C*C_rate_SI ; V_exp + R_times_C*C_rate_SI ; V_nom + R_times_C*C_rate_SI];
x = M_A\b;

Eo = x(1);
A = x(2);
K = x(3);

end