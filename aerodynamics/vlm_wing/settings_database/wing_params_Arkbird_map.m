% Wing parameters (Arkbird)

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

%% basic parameters

% wing span (scalar), in m
prm.b         	= 0.84;
% is the wing symmetrical to the xz-plane (scalar bolean)?
prm.is_symmetrical = true;

%% wing segments (s segments)

% spanwise coordinates of wing parts (1x(s+1) array), in -
prm.eta_segments_wing = [0 1];
% wing chord (1x(s+1) array), in m
prm.c = [0.34 0.16];
% quarter chord line sweepback angle for each section (1xs array), in rad
prm.lambda    	= deg2rad([18]);
% wing section twist (1xs array), in rad
prm.epsilon   	= [0];  
% dihedral angle for each section (1xs array), in rad
prm.nu       	= deg2rad([0]); 

%% wing device segments (d device segments)

% spanwise coordinates of wing device segments (1x(d+1) array), in -
prm.eta_segments_device = [0.000 0.19 1];
% section name (dx? char array)
prm.section = char(["map_NACA_0010_flap_01933"; "map_NACA_0010_flap_01933"]);
% relative flap depth for each device segment (1xd array), in -
prm.flap_depth          = [ 0, 0.1933 ];
% second actuator type (dx? char array)
prm.actuator_2_type = char(repmat(["none"],2,1));
% index of the control input (0 means no control input) (1x(2*d) array), in -  
prm.control_input_index = [ 1 0 0 2; ...
                            0 0 0 0 ];

%% coordinates in reference frame

% wing incidence relative x-y-plane of reference frame, in rad
prm.i = 0;
% wing rotation about x axis of reference frame, in rad
prm.rot_x = 0;
% x coordinate intersection point wing leading edge and x-z-plane, in m
prm.x = 0; 
% z coordinate intersection point wing leading edge and x-z-plane, in m
prm.z = 0; 
