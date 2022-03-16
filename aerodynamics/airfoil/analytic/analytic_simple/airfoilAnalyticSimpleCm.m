function c_m = airfoilAnalyticSimpleCm( airfoil, c_L )
% airfoilAnalyticSimpleCm computes the pitching moment coefficient with
% respect to the 25% chord position for a simple airfoil.
% 
% Inputs:
%   airfoil         simple airfoil struct, see
%                   airfoilAnalyticSimpleLoadParams
%   c_L             lift coefficients, dimensionless
% 
% Outputs:
%   c_m             pitching moment coefficients
% 
% See also:
%   airfoilAnalyticSimpleLoadParams, airfoilAnalyticSimpleCl
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

c_m = airfoil.c_m0 - c_L * (airfoil.x_ac - 0.25);

end