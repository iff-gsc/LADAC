%  ** conventional airplane aerodynamics (simple) parameters **

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

%% fuselage parameters
% fuselage volume, m^3 
fuse.V = 0.0025;
aero.fuse = fuselageLoadParams( 'simpleFuselage_params_default', fuse.V );

%% main wing parameters
aero.wingMain = simpleWingLoadParams( 'simpleWing_params_default' );

%%  horizontal tailplane parameters
aero.wingHtp = simpleWingLoadParams( 'simpleWing_params_default' );

%% vertical tailplane parameters
aero.wingVtp = simpleWingLoadParams( 'simpleWing_params_default' );

%% downwash
wing_main = wingCreate( 'simpleWing_params_default', 40 );
wing_htp = wingCreate( 'simpleWing_params_default', 20 );

aero.downwash = wingGetDownwashDerivs( wing_main, wing_htp );

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
