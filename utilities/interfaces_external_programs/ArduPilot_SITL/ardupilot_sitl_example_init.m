%% set parameters of the spiral

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% time derivative of velocity vector of the body relative to the earth in
% body frame (forward, right, down) for the unsteady acceleration 
% grounded->spiral, in m/s^2
dot_V_Kb_unsteady = [4;0;0];

% second time derivative of the Euler angles for the unsteady acceleration
% grounded->spiral, in rad/s^2
ddot_EulerAngles_unsteady = [0.1;0.05;0];

% second time derivative of the Euler angles during the steady spiral
% maneuver, in rad/s^2
ddot_EulerAngles_steady = [0;0;0.1];

% steady bank angle (this angle is used to stop the unsteady rolling 
% maneuver), in rad
Phi_steady = 0.5;

% steady flight path velocity (this velocity is used to stop the unsteady
% acceleration maneuver), in m/s
V_Kb_steady = 20;

% gravity of Earth, in m/s^2
g = 9.81;

open('ardupilot_sitl_example');

