function prop = propMapLoadParams( filename )
%PROPMAPLOADPARAMS loads a propeller struct.
% 
% Example:
%  prop = propMapLoadParams( 'propMap_params_default' );
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

run(filename);

load('DATA_APC');

prop.map_fit = propMapFitCreate(DATA_APC,prop.name);

end