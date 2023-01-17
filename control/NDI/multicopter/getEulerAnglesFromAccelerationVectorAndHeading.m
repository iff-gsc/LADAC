function [euler_angles, acc_res] = ...
    getEulerAnglesFromAccelerationVectorAndHeading(acc_vec, Psi) %#codegen

% getEulerAnglesFromAccelerationVectorAndHeading computes the euler angles
% that transforms the acceleration vector into the equivalent
% Z-Accerleration in Body-Coordinates for a given acceleration vector in
% the North-East-Down Coordinate-System and heading.
%
% Syntax: [euler_angles, acc_res] = 
%         getEulerAnglesFromAccelerationVectorAndHeading(acc_vec, Psi) 
%
% Inputs:
%   acc_vec       acceleration vector in North East Down (NED) Coordinate
%                 System
%                 (scalar), in m/s^2
%   Psi           heading of the positive X-Axis against North,
%                 (scalar), in rad                            
%
% Outputs:
%   euler_angles  euler angles (pitch, roll, yaw)
%                 (1x3 vec), in rad
%
%   acc_res       accerleration in Z-Axis in Body-System,
%                 (scalar), in m/s^2
%

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Fabian Guecker
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

acc_res = sqrt(acc_vec(1)^2 + acc_vec(2)^2 + acc_vec(3)^2);

% Rotate Coordinate System by Azimuth Angle Psi
point_g_rot_Psi = euler2Dcm([0 0 Psi]) * (-acc_vec);

% Calculate pitch angle Theta
Theta = atan2(point_g_rot_Psi(1), point_g_rot_Psi(3));

% Rotate Coordinate System by Angle Theta
point_g_rot_Theta = euler2Dcm([0 Theta 0]) * point_g_rot_Psi;

% Calculate roll angle Phi
Phi = atan2(-point_g_rot_Theta(2), point_g_rot_Theta(3));

% for testing purposes
% point_kk = euler2Dcm([Phi, Theta, Psi])' * [0 0 norm(acc_vec)]';

% return euler angles
euler_angles = [Phi; Theta; Psi];

end
