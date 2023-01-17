function tailsitter = tailsitterLoadParams( filename )
% the variable loaded from filename must be equal to
% tailsitter_params_default
% 
% Example:
%   airplane = tailsitterLoadParams( ...
%       'tailsitter_params_default' );
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