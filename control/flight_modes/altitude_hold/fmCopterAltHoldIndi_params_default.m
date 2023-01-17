% ** INDI altitude hold flight mode parameters (default) **

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% attitude controller
param.atc = loadParams( 'cntrlAttiRedIndi_params_default' );

% attitude controller
param.psc = loadParams( 'cntrlPosNdi_params_default' );

% propeller control effectiveness
param.cep = loadParams( 'indiCeProp_params_default' );

% body control effectiveness
param.ceb = loadParams( 'indiCeBody_params_default' );

% control allocation
param.ca = loadParams( 'control_allocation_wls_params_default' );

% sensor filter
param.sflt = loadParams( 'indiSensFilt_params_default' );

% motor time constant, in s
param.mtc = 0.0135;

% flight mode sample time, in s
param.ts = 0.0025;
