%% measured values from ahrs object

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% angular velocity of FRD frame relative to the earth represented in FRD
% frame, in rad/s
measure.omega_Kb    = zeros(3,1);
% Euler angles of FRD frame relative to NED frame, in 1
measure.EulerAngles = zeros(3,1);
% quaternion from NED to FRD frame
measure.q_bg        = euler2Quat(measure.EulerAngles);
% measured acceleration represented in NED frame, in m/s^2
measure.a_Kg        = zeros(3,1);
% velocity of FRD frame relative to the earth represented in FRD frame, in
% m/s
measure.V_Kg        = zeros(3,1);
% local position NED, in m
measure.s_Kg        = zeros(3,1);
% global position (latitude, longitude, altitude), in (?,?,m)
measure.lla         = zeros(3,1);
% rangefinders in (m)
measure.rangefinder = zeros(6,1);

%% commanded values
% stick inputs:
% roll command, -1 ... 1
cmd.roll            = 0;
% pitch command, -1 ... 1
cmd.pitch           = 0;
% yaw command, -1 ... 1
cmd.yaw             = 0;
% throttle, -1 ... 1
cmd.thr             = 0;

% initial values:
% position (NED frame) for initializing integrator
cmd.s_Kg_init       = zeros(3,1);
% yaw for initializing integrator
cmd.yaw_init        = 0;

% Parameters for waypoint mission
cmd.mission_change = uint16(0);
cmd.waypoints      = zeros(4,10);
%In this case the maximum number of waypoints is 10. 
cmd.num_waypoints  = uint16(0);

% RC PWM input
cmd.RC_pwm         = zeros(16,1);

%% SampleTime for ArduPilot (s)

cntrl.sample_time   = 1/400;

%% create bus object
struct2bus( measure, 'measureBus' );
struct2bus( cmd, 'cmdBus' );
