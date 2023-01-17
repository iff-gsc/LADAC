function tiltwing = tandemTiltwingLoadParams( filename )
% the variable loaded from filename must be equal to
% tandem_tiltwing_params_default
% 
% Example:
%   tiltwing = tandemTiltwingLoadParams( ...
%       'tandem_tiltwing_params_default' );
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% load parameters from file 
run(filename);

end