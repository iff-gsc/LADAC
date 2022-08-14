% ** INDI altitude hold flight mode parameters (default) **

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% attitude controller
fm_alt_hold.atc = cntrlAttiRedIndiLoadParams( ...
    'cntrlAttiRedIndi_params_default' );

% attitude controller
fm_alt_hold.psc = cntrlPosNdiLoadParams( ...
    'cntrlPosNdi_params_default' );

% control effectiveness
fm_alt_hold.cntrl_effect = cntrlEffectPropLoadParams( ...
    'cntrlEffectProp_params_default' );

% control allocation
fm_alt_hold.ca = controlAllocationWlsLoadParams( ...
    'control_allocation_wls_params_default' );

% sensor filter
fm_alt_hold.sens_filt = indiSensFiltLoadParams( ...
    'indi_sens_filt_params_default' );

% motor time constant, in s
fm_alt_hold.motor_time_constant = 0.0135;

% flight mode sample time, in s
fm_alt_hold.sample_time = 0.0025;
