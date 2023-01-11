%  ** conventional airplane parameters **

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% rigid body parameters
airplane.body = rigidBodyLoadParams( 'rigid_body_params_default' );

% center of gravity in c frame, m
airplane.config.cg = [-1;0;0];
% collision points in c frame, m
airplane.config.hitPoints = [ [0;0;0],[-1.5;0;0],[-0.5;-1;0],[-0.5;1;0] ];

% aerodynamics parameters
airplane.aero = conventionalAirplaneAeroLoadParams( ...
    'params_conventional_airplane_aero_simple_default' );

% propeller parameters
airplane.prop = propMapLoadParams( 'propeller_map_based_params_default' );

% propeller configuration parameters
airplane.prop.config.Pos = [0;0;0];
airplane.prop.config.Rot = eye(3);

% motor parameters
airplane.motor = loadParams( 'motor_bldc_params_default' );

% actuator parameters
airplane.act.ailerons = loadParams('actuators_pt2_params_default');
airplane.act.elevator = loadParams('actuators_pt2_params_default');
airplane.act.rudder = loadParams('actuators_pt2_params_default');
airplane.act.htpTrim = loadParams('actuators_pt2_params_default');

% battery parameters
airplane.bat = 14.8;

% cmd struct/bus
airplane.cmd = struct('aileron_left',0.5,'aileron_right',0.5,'elevator',0.5,'rudder',0.5,'throttle',0,'htp_trim',0.5);
struct2bus(airplane.cmd,'cmdBus');

% reference position
airplane.posRef = posRefLoadParams( 'reference_position_params_default' );

% ground contact parameters
airplane.grnd = groundLoadParams( 'params_ground_default' );

%% initial conditions (IC)
% kinematic rotational velocity, rad/s
airplane.ic.omega_Kb = zeros(3,1);
% quaternion attitude from NED to body frame
airplane.ic.q_bg = [1;0;0;0];
% kinematic velocity in body frame, m/s
airplane.ic.V_Kb = [20;0;0];
% NED position relative to posRef, m
airplane.ic.s_Kg = zeros(3,1);
% motor angular velocity, rad/s
airplane.ic.motor_speed = 0;