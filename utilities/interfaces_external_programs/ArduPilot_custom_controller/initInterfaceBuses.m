function [] = initInterfaceBuses( varargin )


% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************


if isempty(varargin)
    data_type = 'single';
else
    data_type = varargin{1};
end

%% measured values from ahrs object

% angular velocity of FRD frame relative to the earth represented in FRD
% frame, in rad/s; This unfiltered (and noisy) signal is obtained on a
% custom way and should be preferred over omega_Kb and then processed by a
% custom low-pass filter. In contrast to omega_Kb the time-derivative of
% the filtered Omega_Kb_raw is NOT noisy.
measure.Omega_Kb_raw = zeros(3,1);
% angular velocity of FRD frame relative to the earth represented in FRD
% frame as used internally in ArduPilot, in rad/s. This signal is filtered
% by a low-pass filter with cutoff frequency INS_GYRO_FILTER. However, 
% Omega_Kb_raw should be perferred because the time-derivative of omega_Kb
% is still noisy due to unknown reasons.
measure.omega_Kb    = zeros(3,1);
% Euler angles of FRD frame relative to NED frame, in rad (predicted and
% thus almost without delay)
measure.EulerAngles = zeros(3,1);
% quaternion from NED to FRD frame (predicted and thus almost without
% delay)
measure.q_bg        = euler2Quat(measure.EulerAngles);
% measured acceleration represented in NED frame, in m/s^2 (x and y
% component almost without delay; z component seems to be delayed by
% INS_ACCEL_FILTER)
measure.a_Kg        = zeros(3,1);
% velocity of FRD frame relative to the earth represented in FRD frame, in
% m/s (predicted and thus almost without delay)
measure.V_Kg        = zeros(3,1);
% local position NED, in m (there is a delay that should be elaborated in
% the future)
measure.s_Kg        = zeros(3,1);
% local position NED w.r.t. origin, in m (there is a delay that should be elaborated in
% the future)
measure.s_Kg_origin = zeros(3,1);
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

%% create bus object

if strcmp(data_type,'single')
    measure = structDouble2Single(measure);
    cmd = structDouble2Single(cmd);
end

struct2bus( measure, 'measureBus' );
struct2bus( cmd, 'cmdBus' );

end
