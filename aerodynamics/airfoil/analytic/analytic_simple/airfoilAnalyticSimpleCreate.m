function airfoil = airfoilAnalyticSimpleCreate( filename )
% airfoilAnalyticSimpleCreate creates a simple airfoil struct.
% 
% Inputs:
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

airfoil = airfoilAnalyticSimpleInit( );

airfoil = airfoilAnalyticSimpleLoadParams(airfoil,filename);

end