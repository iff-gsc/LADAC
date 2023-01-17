function airfoil = airfoilAnalyticSimpleInit( )
% airfoilAnalyticSimpleInit initializes a simple airfoil struct.
% 
% Outputs:
%   airfoil         simple airfoil struct as defined by this function
% 
% See also:
%   airfoilAnalyticSimpleLoadParams, airfoilAnalyticSimpleCl
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% lift curve slope, in 1/rad
airfoil.alpha_0    	= 0;
% zero lift angle of attack, in rad
airfoil.c_L_alpha 	= 0;

% drag coefficient at zero angle of attack, dimensionless
airfoil.c_D0        = 0;
% c_D1, in 1/rad
airfoil.c_D1        = 0;
% c_D2, in 1/rad^2
airfoil.c_D2        = 0;

% pitching moment coefficient at zero angle of attack, dimensionless
airfoil.c_m0        = 0;
% pitching moment coefficient slope, in 1/rad
airfoil.x_ac   = 0;

end