function micro_tab = airfoilMicroTabLoadParams( filename )
% airfoilMicroTabLoadParams loads a parameter struct for a micro-tab
% 
% Example:
%   airfoil = airfoilMicroTabLoadParams( 'airfoilMicroTab_params_F15_90' )

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************


run( filename );

end