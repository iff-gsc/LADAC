function c_D = airfoilAnalyticSimpleCd( airfoil, alpha )
% airfoilAnalyticSimpleCd computes the drag coefficient for a simple
% airfoil.
% 
% Inputs:
%   airfoil         simple airfoil struct, see
%                   airfoilAnalyticSimpleLoadParams
%   alpha           angle of attack, in rad
% 
% Outputs:
%   c_D             drag coefficients
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

c_D = airfoil.c_D0 + airfoil.c_D1*alpha + airfoil.c_D2*alpha.^2;

end