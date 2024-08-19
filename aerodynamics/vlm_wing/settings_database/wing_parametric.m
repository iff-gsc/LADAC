function prm = wing_parametric( aspect_ratio, taper_ratio, sweep_angle, twist_angle, varargin )
% Wing parameters (for different basic parameters for verification)

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% spanwise coordinates of wing device segments (1x(d+1) array), in -
prm.eta_segments_device = [0 1];
% relative flap depth for each device segment (1xd array), in -
prm.flap_depth          = [0];
% index of the control input (0 means no control input) (1x(2*d) array), in -  
prm.control_input_index = [0,0];
% section name (dx? char array)
prm.section = 'airfoilAnalyticSimple_params_default';
% second actuator type (dx? char array)
prm.actuator_2_type = 'none';

for i = 1:length(varargin)
    if strcmp(varargin{i},'Flaps')
        prm.eta_segments_device = varargin{i+1};
        prm.flap_depth = varargin{i+2};
        d = length(prm.flap_depth);
        prm.section = repmat(prm.section,d,1);
        prm.actuator_2_type = repmat(prm.actuator_2_type,d,1);
        flap_depth_2 = [ flip(prm.flap_depth), prm.flap_depth ];
        prm.control_input_index = zeros(1,2*d);
        cntrl_input_idx = 1;
        for j = 1:2*d
            if flap_depth_2(j) > 0
                prm.control_input_index(j) = cntrl_input_idx;
                cntrl_input_idx = cntrl_input_idx + 1;
            end
        end

    end
end

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