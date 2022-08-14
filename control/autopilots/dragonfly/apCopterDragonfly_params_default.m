% ** copter autopilot dragonfly parameters (default) **

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% attitude controller
ap_dragonfly.atc = cntrlAttiRedIndiLoadParams( ...
    'cntrlAttiRedIndi_params_default');

% position position controller
ap_dragonfly.psc = cntrlPosNdiLoadParams( ...
    'cntrlPosNdi_params_default' );

% guidance
ap_dragonfly.traj = trajLoadParams( ...
    'traj_params_default');

% control effectiveness
ap_dragonfly.cntrl_effect = cntrlEffectPropLoadParams( ...
    'cntrlEffectProp_params_default' );

% control allocation
ap_dragonfly.ca = controlAllocationWlsLoadParams( ...
    'control_allocation_wls_params_quadcopter' );

% sensor filter
ap_dragonfly.sens_filt = indiSensFiltLoadParams( ...
    'indi_sens_filt_params_default' );

% motor time constant, in s
ap_dragonfly.motor_time_constant = 0.028;

% flight mode sample time, in s
ap_dragonfly.sample_time = 0.0025;