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

% propeller control effectiveness
param.cep = loadParams( 'indiCeProp_params_default' );

% body control effectiveness
param.ceb = loadParams( 'indiCeBody_params_default' );

% control allocation
param.ca = controlAllocationWlsLoadParams( ...
    'control_allocation_wls_params_quadcopter' );

% sensor filter
param.sflt = indiSensFiltLoadParams( ...
    'indiSensFilt_params_default' );

% motor time constant, in s
param.mtc = 0.028;

% flight mode sample time, in s
param.ts = 0.0025;
