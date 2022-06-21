function c_D = airfoilAnalytic0515AlCdNorm(fcd,x,alpha_q,c_d_q)
% airfoilAnalytic0515AlCdNorm is the normalized analytic function for the
% drag coefficient for angle of attack / alpha airfoilAnalytic0515AlCd
%   This function is used for fitting because it yields a better fit.
% 
% Inputs:
%   fcd             analytic function parameters array (see outputs of
%                   airfoilAnalytic0515AlFit)
%   x               angle of attack (row vector)
%   alpha_q         query points of angle of attack
%   c_d_q           query points of drag coefficient
% 
% Outputs:
%   c_D             normalized drag coefficient
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************


c_D = 1 ./ interp1(alpha_q(:)',c_d_q(:)',x,'pchip') .* ...
    airfoilAnalytic0515AlCd( fcd, x );


end