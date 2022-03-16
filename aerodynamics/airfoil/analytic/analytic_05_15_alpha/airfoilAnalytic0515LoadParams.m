function airfoil = airfoilAnalytic0515LoadParams( filename )
% airfoilAnalytic0515LoadParams loads a parameter struct for an analytic
% airfoil computation.
% 
% Example:
%   airfoil = airfoilAnalytic0515LoadParams( 'airfoilAnalytic0515_params_F15' )

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

run( filename );

end