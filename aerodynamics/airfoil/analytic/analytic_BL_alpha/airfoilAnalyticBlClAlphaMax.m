function [c_N_alpha,alpha_0] = airfoilAnalyticBlClAlphaMax(fcl)
% airfoilAnalyticBlClAlphaMax returns the maximum lift curve slope
% (approximately) and the zero lift angle of attack (approximately).
%   Depending on the analytic function coefficients and on the Mach number,
%   the maximum lift curve slope and the zero lift angle of attack are
%   different.
%   As the function is analytic, the maximum value is tried to be
%   determined analytically as well because of efficiency.
%   However, no analytic solution could be found. But the implemented
%   analytic solution returns a value that is very close to the actual
%   value.
% 
% Inputs:
%   fcl         coefficients of the analytic lift curve (see outputs of
%           	airfoilAnalyticBlAlCn)
% 
% Outputs:
%   c_N_alpha 	maximum lift curve slope (1xN array), in 1/deg
%   alpha0      zero lift angle of attack (1xN array), in deg
% 

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

c_N_alpha = fcl(1);
alpha_0 = fcl(2);

end