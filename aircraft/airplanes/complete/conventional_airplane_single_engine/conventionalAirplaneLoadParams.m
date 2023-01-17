function airplane = conventionalAirplaneLoadParams( filename )
% the variable loaded from filename must be equal to
% params_conventional_airplane_aero_simple_default
% 
% Example:
%   airplane = conventionalAirplaneLoadParams( ...
%       'params_conventional_airplane_default' );
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