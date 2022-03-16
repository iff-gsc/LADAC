%  ** conventional airplane aerodynamics (simple) parameters **

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

%% fuselage parameters
aero.fuse = fuselageLoadParams( 'params_aero_fuselage_default' );

%% main wing parameters
aero.wingMain = simpleWingLoadParams( 'params_aero_simple_wing_default' );

%%  horizontal tailplane parameters
aero.wingHtp = simpleWingLoadParams( 'params_aero_simple_wing_default' );

%% vertical tailplane parameters
aero.wingVtp = simpleWingLoadParams( 'params_aero_simple_wing_default' );

%% configuration parameters
% incidence angle of main wing, rad
aero.config.wingMainIncidence = 0;
% position of main wing in c frame, m
aero.config.wingMainPos = [ -1; 0; 0 ];
% position of horizontal tailplane in c frame, m
aero.config.wingHtpPos = [ -2; 0; 0 ];
% position of vertical tailplane in c frame, m
aero.config.wingVtpPos = [ -2; 0; 0 ];
% rotation matrix of vertical tailplane (relative to c frame)
aero.config.wingVtpRot = euler2Dcm( [-pi/2;0;0] );
