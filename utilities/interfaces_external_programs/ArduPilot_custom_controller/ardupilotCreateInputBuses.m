function [] = ardupilotCreateInputBuses( varargin )
% ardupilotCreateInputBuses create Simulink bus objects "cmdBus" and
% "measureBus" for ArduPilot custom controller
%   The ArduPilot custom controller interface needs the buses "cmdBus" and
%   "measureBus" to be in the base workspace, see
%   ArduCopter_TemplateController.slx and ArduPlane_ManualMode.slx.
% 
% Syntax:
%   ardupilotCreateInputBuses()
%   ardupilotCreateInputBuses(data_type)
% 
% Inputs:
%   data_type           (optional) data type of the floating point
%                       variables in the "cmdBus" and "measureBus"
%                       ('single' (default) or 'double')
% 
% Outputs:
%   none (but two Simulink bus objects named "cmdBus" and "measureBus" will
%   be assigned to the base workspace)
% 
% See also:
%   ardupilotCreateLogBus

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
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
% frame, in rad/s; This signal is unfiltered (and noisy) and should be
% processed by a custom low-pass filter.
measure.omega_Kb = zeros(3,1);
% Euler angles of FRD frame relative to NED frame, in rad (predicted and
% thus almost without delay)
measure.EulerAngles = zeros(3,1);
% quaternion from NED to FRD frame (predicted and thus almost without
% delay)
measure.q_bg        = euler2Quat(measure.EulerAngles);
% measured acceleration represented in NED frame, in m/s^2 (delayed by
% INS_ACCEL_FILTER; contains the acceleration due to gravity and is at
% standstill [0;0;-9.81])
measure.a_g         = zeros(3,1);
% measured acceleration represented in FRD frame, in m/s^2
% (contains the acceleration due to gravity and is at standstill
% M_bg*[0;0;-9.81], where M_bg is the rotation matrix from g to b frame)
measure.a_b         = zeros(3,1);
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
% battery voltage in volt
measure.V_bat       = 0;
% motor angular velocities, in rad/s
measure.omega_mot   = zeros(4,1);
% measured airspeed from airspeed sensor, in m/s
measure.airspeed = 0;

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
