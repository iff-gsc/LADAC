function c_D = airfoilAnalytic9090AlCdNorm(beta,x,alpha_q,c_d_q)
% analytic function for the drag coefficient for angle of attack / alpha 
% NORMALIZED
% 
% Inputs:
%   beta            parameter vector
%   xy              concentrated vectors of angle of attack (first row)
%                   and drag coefficient (second row)
% 
% Outputs:
%   c_D             drag coefficient vector
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************


c_D = 1 ./ interp1(alpha_q,c_d_q,x(1,:),'pchip') .* ...
    airfoilAnalytic9090AlCd( beta, x );


end