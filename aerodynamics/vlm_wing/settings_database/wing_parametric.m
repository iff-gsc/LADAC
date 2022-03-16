function prm = wing_parametric( aspect_ratio, taper_ratio, sweep_angle, twist_angle )
% Wing parameters (for different basic parameters for verification)

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

%% pre compute
b = 10;
S = b^2 / aspect_ratio;
c_0 = (2*S) / (b *(1+taper_ratio));


%% basic parameters

% wing span (scalar), in m
prm.b         	= b;
% is the wing symmetrical to the xz-plane (scalar bolean)?
prm.is_symmetrical = true;

%% wing segments (s segments)

% spanwise coordinates of wing parts (1x(s+1) array), in -
prm.eta_segments_wing = [0 1];
% wing chord (1x(s+1) array), in m
prm.c = [ c_0  c_0*taper_ratio ];
% quarter chord line sweepback angle for each section (1xs array), in rad
prm.lambda    	= [sweep_angle];
% wing section twist (1xs array), in rad
prm.epsilon   	= [twist_angle];  
% dihedral angle for each section (1xs array), in rad
prm.nu       	= [0]*pi/180;  

%% wing device segments (d device segments)

% spanwise coordinates of wing device segments (1x(d+1) array), in -
prm.eta_segments_device = [0 1];
% section name (dx? char array)
prm.section = char(repmat(["airfoilAnalyticSimple_params_default"],1,1));
% relative flap depth for each device segment (1xd array), in -
prm.flap_depth          = [0];
% second actuator type (dx? char array)
prm.actuator_2_type = char(repmat(["none"],5,1));
% index of the control input (0 means no control input) (1x(2*d) array), in -  
prm.control_input_index = [1 2];

%% coordinates in reference frame

% wing incidence relative x-y-plane of reference frame, in rad
prm.i = 0;
% wing rotation about x axis of reference frame, in rad
prm.rot_x = 0;
% x position of the wing (leading edge at wing root) in reference frame (scalar), in m
prm.x = 0; 
% z position of the wing (leading edge at wing root) in reference frame (scalar), in m
prm.z = 0;

end