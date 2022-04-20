function fm_alt_hold = fmCopterAltHoldIndiLoadParams( filename )
% fmCopterAltHoldIndiLoadParams loads an INDI altitude hold
%   flight mode (for multicopter or similar) struct
% 
% Example:
%  fm_alt_hold = fmCopterAltHoldIndiLoadParams( ...
%   'fmCopterAltHoldIndi_params_default' );
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

run(filename);

end