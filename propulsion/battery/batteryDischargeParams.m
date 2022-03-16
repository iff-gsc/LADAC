function [Eo, A, K] = batteryDischargeParams( points )
% batteryDischargeParams computes the parameters of the discharge curve
%   
%   This function computes the parameters of the discharge curve according
%   to the model of Trembley (2009) [TO DO].
%
% Syntax:  [Eo, A, K] = batteryDischargeParams( points );
%
% Inputs:
%   points      a vector with 8 entries containing 4 experimental points on
%               the discharge curve (vector)
%
% Outputs:
%   Eo          a parameter of the discharge curve (scalar)
%   A           a parameter of the discharge curve (scalar)
%   K           a parameter of the discharge curve (scalar)
%
%
% See also: batteryAverageParams
%

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% the experimental points on the discharge curve
SoC_full = points(1);
SoC_nom = points(2);
SoC_exp = points(3);
V_full = points(4);
V_exp = points(5);
V_nom = points(6);
C_rate = points(7);
R_over_C = points(8);

% equation to calculate the 3 parameters Eo, A, K of the discharge curve
% according to Trembley
M_A = [1, 1, 0 ; ...
    1, exp(-3), -SoC_full/(SoC_full-SoC_exp)*(SoC_exp+C_rate*0) ; ...
    1, exp(-3*SoC_nom/SoC_exp), -SoC_full/(SoC_full-SoC_nom)*(SoC_nom + C_rate*0)];
b = [V_full + R_over_C*C_rate ; V_exp + R_over_C*C_rate ; V_nom + R_over_C*C_rate];
x = M_A\b;

Eo = x(1);
A = x(2);
K = x(3);

end