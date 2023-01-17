
% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Fabian Guecker
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

%% Clear workspace and console
clc;
clear;

%% Create waypoints

waypoints = [1 -1 0.2; 2 0 0.1; 1 1 0; -1 -1 0; -2 0 0.1; -1 1 0.2]'*50;
cycle = true;

%% Generate empty trajectory struct and bus definition for simulink 

% degree of polynomial
degree = 5;

% Create emptry trajectory with a maximum space for 10 waypoints
traj_size = size(waypoints,2);
[traj_empty] = trajInit(traj_size, degree);

% Create simulink bus definition
trajectoryBus = struct2bus_(traj_empty);

% compute trajectory
traj = trajFromWaypoints(traj_empty, waypoints, degree, cycle);

%% Calculate inital values for smooth simulation

% inital velocity (norm)
inital_velo  = 20;
g = 9.81;

% inital positon
inital_point = waypoints(:,1);

% inital velocity vector and acceleration
[active_section, Error, t] = trajGetMatch(traj, inital_point);
traj_section = trajGetSection(traj,active_section);
[T, B, N] = trajSectionGetFrenetSerretWithGravity(traj_section,inital_velo, g, t);
inital_vel_vec = inital_velo * T;

% Calculate inital attitude
initalSerretFrame = [T, B, -N];
inital_quaternion = rotm2quat(initalSerretFrame);

%% Plot trajectory with expected acceleration for constant given velocity

figure(1);
trajPlot(traj, inital_velo, g);
view([-1 -1.2 0.5])
axis equal


open('traj_rigid_body_example');


