% ** tandem tiltwing parameters (Changyucopter) **

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

%% rigid body parameters

tiltwing.body = rigidBodyLoadParams( 'rigid_body_params_default' );


%% set configuration parameters

tiltwing.config.xyz_c_cg = [-0.56; 0; 0.02];                         % center of mass in c frame

% configuration
tiltwing.config.xyz_c_tp1 = [-0.2; 0; 0.04];                     % position of the front tilt wing in c frame
tiltwing.config.xyz_c_tp2 = [-0.92; 0; -0.04];                   % position of the aft tilt wing in c frame

tiltwing.config.xyz_mi_M1 = [0.06; -0.65; 0.02];                 % 1st motor position in tilt frame
tiltwing.config.xyz_mi_M2 = [0.06; 0.65; 0.02];                  % 2nd motor position in tilt frame
tiltwing.config.xyz_mi_M3 = [0.06; 0.45; 0.02];                  % 3rd motor position in tilt frame
tiltwing.config.xyz_mi_M4 = [0.06; -0.45; 0.02];                 % 4th motor position in tilt frame

tiltwing.config.propDir = [-1;1;-1;1];                    % 1: pos. about propeller axis, -1: neg.

% renaming
tiltwing.config.xyz_mi_M_i = [tiltwing.config.xyz_mi_M1 tiltwing.config.xyz_mi_M2 tiltwing.config.xyz_mi_M3 tiltwing.config.xyz_mi_M4];

% collision points
tiltwing.config.collision_points_c = [ ...
    [ -0.25; -0.2; 0.25 ], ...
    [ -0.25; 0.2; 0.25 ], ...
    [ -0.85; -0.2; 0.25 ], ...
    [ -0.85; 0.2; 0.25 ] ];

%% propellers, motors, battery

tiltwing.map_grid = propMapLoadParams( 'propeller_map_based_params_default' );
tiltwing.motor = motorLoadParams( 'motor_bldc_params_default' );
tiltwing.bat = batteryLoadParams( 'battery_params_default' );

%% fuselage parameters

tiltwing.aero.fuselage = fuselageLoadParams( 'params_aero_fuselage_default' );

% neutral point of the fuselage in c frame
tiltwing.aero.fuselage.xyz_c_np_fuselage = [-0.3; 0; 0];

%% parameters of the first wing

tiltwing.aero.wing_1 = simpleWingLoadParams( 'params_aero_simple_wing_default' );
tiltwing.aero.alphaw_alphai = -1.2;

tiltwing.aero.wing_2 = simpleWingLoadParams( 'params_aero_simple_wing_default' );

%% actuator parameters

tiltwing.act.elevon = actuatorsLoadParams( 'actuators_pt2_params_default' );
tiltwing.act.tilt = actuatorsLoadParams( 'actuators_pt2_params_default' );

%% initial conditions
% kinematic rotational velocity, rad/s
tiltwing.ic.p = 0;
tiltwing.ic.q = 0;
tiltwing.ic.r = 0;
% Euler angles attitude from NED to body frame
tiltwing.ic.Phi = 0;
tiltwing.ic.Theta = 0;
tiltwing.ic.Psi = 0;
% kinematic velocity in body frame, m/s
tiltwing.ic.u = 0;
tiltwing.ic.v = 0;
tiltwing.ic.w = 0;
% NED position relative to posRef, m
tiltwing.ic.x = 0;
tiltwing.ic.y = 0;
tiltwing.ic.z = 0;
% motor angular velocity, rad/s
tiltwing.ic.omega = zeros(4,1);
% tilt angles, deg
tiltwing.ic.tilt_angle_1 = 90;
tiltwing.ic.tilt_angle_2 = 90;
% elevon angles, deg
tiltwing.ic.elevon_1 = 0;
tiltwing.ic.elevon_2 = 0;
% downwash
tiltwing.ic.downwash = 0;

%% reference position

tiltwing.pos_ref = posRefLoadParams( 'reference_position_params_default' );