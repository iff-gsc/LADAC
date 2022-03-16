function fm_loiter = fmCopterLoiterIndiLoadParams( filename )
% fmCopterLoiterIndiLoadParams loads a loiter indi controller (for
%   multicopters or similar) parameters struct
% 
% Example:
%  fm_atti = fmCopterLoiterIndiLoadParams( ...
%   'fm_copter_loiter_indi_params_default' );
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

run(filename);

end