% ** INDI loiter flight mode parameters (default) **

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% attitude controller
param.atc = cntrlAttiRedIndiLoadParams( ...
    'cntrlAttiRedIndi_params_default');

% position position controller
param.psc = cntrlPosNdiLoadParams( ...
    'cntrlPosNdi_params_default' );

% control effectiveness
param.cntrl_effect = controlEffectivenessLoadParams( ...
    'control_effectiveness_params_default' );

% control allocation
param.ca = controlAllocationWlsLoadParams( ...
    'control_allocation_wls_params_quadcopter' );

% sensor filter
param.sens_filt = indiSensFiltLoadParams( ...
    'indi_sens_filt_params_default' );

% motor time constant, in s
param.motor_time_constant = 0.028;

% flight mode sample time, in s
param.sample_time = 0.0025;
