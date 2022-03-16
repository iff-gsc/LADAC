% Example script

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% add to path TiGL and TiXI
addPathTiGL('2.2.3')

% Get handles from TiXI and TiGL
CPACS_file_name = 'SE2A_AC_Design_MR_V1_BwdSweep_CPACS.xml';
CPACS_file_path = which(CPACS_file_name);
tixiHandle = tixiOpenDocument( CPACS_file_path );
tiglHandle = tiglOpenCPACSConfiguration( tixiHandle, '' );

%% Lifting Line Theory

wing = wingCreateWithCPACS( tiglHandle, 1, 74 );

%% define current wing state

% define current rigid body state
alpha = deg2rad(3);
beta = 0;
V = 100;
omega = [0;0;0];
h = 100;

% define actuator states
actuatorsLeft = [ 0, -5, 0 ]*0;
actuatorsRight = [ 0, 5, 0 ]*0;

%% compute state

wing = wingSetState(wing, alpha, beta, V, omega, ...
    [ actuatorsLeft, actuatorsRight ], zeros(1,6), zeros(3,1) );

%% get aerodynamic forces and moments
XYZ_i_a         = wingGetLocalForce( wing );
LMN_i_a         = wingGetLocalMoment( wing );
XYZ_a           = wingGetGlobalForce( wing );
LMN_a           = wingGetGlobalMoment( wing );

%% plot results

y = -wing.state.aero.coeff_loc.c_XYZ_b(3,:);
% y = wing.state.aero.circulation.gamma_bound;
plot( wing.geometry.ctrl_pt.pos(2,:)/wing.params.b*2, y );
xlabel('y, m')
ylim([min(0,min(y)) inf])
ylabel('c_L, 1')
grid on