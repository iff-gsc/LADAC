function fm_atti = fmCopterAttiIndiLoadParams( filename )
% fmCopterAttiIndiLoadParams loads a reduced attitude controller
%   struct
% 
% Example:
%  fm_atti = fmCopterAttiIndiLoadParams( ...
%   'fmCopterAttiIndi_params_default' );
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

run(filename);

fm_atti.cntrl_effect.ny_du_red = fm_atti.cntrl_effect.ny_du_red(1:3,:);
fm_atti.cntrl_effect.ny_du_dt = fm_atti.cntrl_effect.ny_du_dt(1:3,:);

fm_atti.ca.W_v = fm_atti.ca.W_v(1:3,1:3);

end