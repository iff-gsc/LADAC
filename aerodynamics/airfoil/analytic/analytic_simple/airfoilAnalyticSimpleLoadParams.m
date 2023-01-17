function airfoil = airfoilAnalyticSimpleLoadParams( airfoil, filename )
% airfoilAnalyticSimpleLoadParams loads parameters from a parameters file
% to a simple airfoil struct.
% 
% Inputs:
%   airfoil         simple airfoil struct, see airfoilAnalyticSimpleInit
%   filename        parameters filename (string), e.g.
%                   'airfoilAnalyticSimple_params_default'
% 
% Outputs:
%   airfoil         simple airfoil struct, see airfoilAnalyticSimpleInit
% 
% See also:
%   airfoilAnalyticSimpleInit, airfoilAnalyticSimpleCl
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

run( filename );

airfoil.alpha_0     = alpha_0;
airfoil.c_L_alpha   = c_L_alpha;
airfoil.c_D0        = c_D0;
airfoil.c_D1        = c_D1;
airfoil.c_D2        = c_D2;
airfoil.c_m0        = c_m0;
airfoil.x_ac        = x_ac;

end