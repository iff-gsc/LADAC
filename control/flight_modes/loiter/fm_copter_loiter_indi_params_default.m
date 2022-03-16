% ** INDI loiter flight mode parameters (default) **

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% altitude hold flight mode
 fm_loiter.fm_alt = fmCopterAltHoldIndiLoadParams( ...
  'fm_copter_alt_hold_indi_params_default' );

% horizontal position controller
fm_loiter.horiz_pos_cntrl = cntrlHorizPosIndiLoadParams( ...
    'cntrl_horiz_pos_indi_params_default' );

% sensor filter
fm_loiter.sens_filt = indiSensFiltLoadParams( ...
    'indi_sens_filt_params_default' );

% flight mode sample time, in s
fm_loiter.sample_time = 0.0025;