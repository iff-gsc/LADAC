% Wing parameters (B737-300)

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

%% basic parameters

% wing span (scalar), in m
prm.b         	= 28.89;
% is the wing symmetrical to the xz-plane (scalar bolean)?
prm.is_symmetrical = true;

%% wing segments (s segments)

% spanwise coordinates of wing parts (1x(s+1) array), in -
prm.eta_segments_wing = [0 0.3358 1];
% wing chord (1x(s+1) array), in m
prm.c = [6.66 3.97 1.37];
% quarter chord line sweepback angle for each section (1xs array), in rad
prm.lambda    	= [23 26]*pi/180;
% wing section twist (1xs array), in rad --> to do: correct twist
prm.epsilon   	= [0 0];  
% dihedral angle for each section (1xs array), in rad
prm.nu       	= [2 5]*pi/180;  

%% wing device segments (d device segments)

% spanwise coordinates of wing device segments (1x(d+1) array), in -
prm.eta_segments_device = [0.000 0.132 0.289 0.389 0.745 0.936 1];
% section name (dx? char array)
prm.section = char(repmat("airfoilAnalyticSimple_params_default",6,1));
% relative flap depth for each device segment (1xd array), in -
prm.flap_depth = [ 0, 0.2, 0.2, 0, 0.2, 0 ];
% second actuator type (dx? char array)
prm.actuator_2_type = char(repmat(["none"],6,1));
% index of the control input (0 means no control input) (1x(2*d) array), in -  
prm.control_input_index = [ 0 1 0 2 3 0 0 4 5 0 6 0 ];

%% coordinates in reference frame

% wing incidence relative x-y-plane of reference frame, in rad
prm.i        	= 1*pi/180;
% wing rotation about x axis of reference frame, in rad
prm.rot_x = 0;
% x position of the wing (leading edge at wing root) in reference frame (scalar), in m
prm.x            = -12.2038; 
% z position of the wing (leading edge at wing root) in reference frame (scalar), in m
prm.z            =   1.3649; 
