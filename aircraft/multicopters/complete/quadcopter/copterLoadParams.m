function copter = copterLoadParams( filename )
% copterLoadParams loads a quadcopter parameters struct.
% 
% Example:
%  copter = copterLoadParams( 'copter_params_default' );
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

run(filename);

% initialize propeller map from propeller name
load('DATA_APC');
copter.prop.map_fit = propMapFitCreate( DATA_APC, copter.prop.name );

end