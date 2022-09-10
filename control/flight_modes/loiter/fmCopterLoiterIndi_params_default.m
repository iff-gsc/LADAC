% ** INDI loiter flight mode parameters (default) **

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% attitude controller
fm_loiter.atc = cntrlAttiRedIndiLoadParams( ...
    'cntrlAttiRedIndi_params_default');

% position position controller
fm_loiter.psc = cntrlPosNdiLoadParams( ...
    'cntrlPosNdi_params_default' );

% propeller control effectiveness
fm_loiter.cep = loadParams( 'indiCeProp_params_default' );

% body control effectiveness
fm_loiter.ceb = loadParams( 'indiCeBody_params_default' );

% control allocation
fm_loiter.ca = controlAllocationWlsLoadParams( ...
    'control_allocation_wls_params_quadcopter' );

% sensor filter
fm_loiter.sflt = indiSensFiltLoadParams( ...
    'indiSensFilt_params_default' );

% motor time constant, in s
fm_loiter.mtc = 0.028;

% flight mode sample time, in s
fm_loiter.ts = 0.0025;