% Example script for trajectory generation 

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Fabian Guecker
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% Clean up
clc;
clear;

% degree of polynominals used for each direction of the trajectory
degree = 5;

% constant velocity for the trajectory
inital_velo = 15;

% acceleration in m/s^2 due to earth gravitation
g = 9.81;

for trajectory_example_no = 1:4
    
    % Eight
    if trajectory_example_no == 1
        waypoints = [1 -1 0.2; 2 0 -0.5; 1 1 0; -1 -1 0; -2 0 -0.5; -1 1 0.2]'*15;
        cycle = true;
    end
    
    % Circle
    if trajectory_example_no == 2
        waypoints = [0 -1 0; 1 0 0 ; 0 1 0; -1 0 0]'*20;
        cycle = true;
    end
    
    % Spiral
    if trajectory_example_no == 3
        turns = 4;
        segments = 5;
        t = 0:pi/segments:turns*2*pi-pi/segments;
        waypoints = [sin(t);cos(t);t/10];
        cycle = true;
    end
    
    % Straight line in 3D
    if trajectory_example_no == 4
        waypoints = [1 1 1; 2 2 2; 3 3 3; 4 4 4]';
        cycle = false;
    end
    
    % Create empty trajectory
    traj_size = size(waypoints,2);
    traj = trajInit(traj_size);
    
    % Compute trajectory
    traj = trajFromWaypoints(traj, waypoints, degree, cycle)
    
    % Plot trajectory
    figure(trajectory_example_no)
    trajPlot(traj, inital_velo, g);
    
end
