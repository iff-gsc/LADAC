%  ** Parameter file of quadcopter (default) **

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

%% body parameters
% mass of vehicle, in kg
copter.body.m = 0.5;
% inertial matrix of vehicle, in kg.m^2
copter.body.I = [0.00192, 0, 0; ...
            0, 0.00185, 0; ...
            0, 0, 0.00334];

%% configuration parameters
% center of gravity position in c frame, in m
copter.config.CoG_Pos_c = [ 0; 0; 0 ];
% position of all propellers in c frame, in m
bx = 0.088;
by = 0.115;
copter.config.propPos_c = [ bx, -by, 0; ...
                            bx, by, 0; ...
                            -bx, by, 0; ...
                            -bx, -by, 0 ]';
% direction of all propeller rotations, -1: right, 1 left
copter.config.propDir = [ 1; -1; 1; -1 ];
% orientation of the motors relative to the body
copter.config.M_b_prop1 = euler2Dcm([0,-pi/2,0]);
copter.config.M_b_prop2 = euler2Dcm([0,-pi/2,0]);
copter.config.M_b_prop3 = euler2Dcm([0,-pi/2,0]);
copter.config.M_b_prop4 = euler2Dcm([0,-pi/2,0]);
% TODO
% hit points for ground model
copter.config.hitPoints_c = [ copter.config.propPos_c + [ 0; 0; 0.13 ], ...
                           copter.config.propPos_c + [ 0; 0; -0.03 ] ];

%% propeller parameters
% propeller moment of inertia, kg.m^2
copter.prop.I = 1.1e-5;
% copter.prop.I = copter.prop.I * 9;
% propeller name specifying propeller map (name must be inside database)
copter.prop.name = '6x3';

%% motor parameters
% torque constant of the motor (KT=60/(2*pi*KV)), N.m/A
% with KV = 1280 RPM/V
copter.motor.KT = 60/(2*pi*1280);
% motor internal resistance, Ohm
copter.motor.R = 0.07;

%% battery parameters
% battery voltage, V
copter.bat.V = 11.1;

% TODO
%% aerodynamics
% reference surface of multicopter, in m^2
copter.aero.S = 0.018;
% minimum drag coefficient, in 1
copter.aero.C_Dmin = 1.3;
% maximum drag coefficient, in 1
copter.aero.C_Dmax = 2.0;
% maximum lift coefficient (whole vehicle), in 1
copter.aero.C_Lmax = 0.1;
% center of pressure (x_b, y_b direction), in m
copter.aero.CoP_xy = 0.03;
% center of pressure (z_b direction), in m
copter.aero.CoP_z = 0.01;
% damping moment coefficient for rotation, in 1/m
copter.aero.rate_damp = -0.011;
