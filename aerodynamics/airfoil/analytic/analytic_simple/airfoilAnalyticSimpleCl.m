function c_L = airfoilAnalyticSimpleCl( airfoil, alpha, varargin )
% airfoilAnalyticSimpleCl computes the lift coefficient for a simple
% airfoil.
% 
% Inputs
%   airfoil         simple airfoil struct, see
%                   airfoilAnalyticSimpleLoadParams
%   alpha           angle of attack, in rad
%   Ma              Mach number
% 
% Outputs
%   c_L             lift coefficients
% 
% Syntax:
%   c_L = airfoilAnalyticSimpleCl( airfoil, alpha )
%   c_L = airfoilAnalyticSimpleCl( airfoil, alpha, Ma )
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

if isempty(varargin)
    Ma = 0;
else
    Ma = varargin{1};
end

beta = 1 ./ sqrtReal( 1 - Ma.^2 );
c_L = airfoil.c_L_alpha * beta .* ( alpha - airfoil.alpha_0 );

end