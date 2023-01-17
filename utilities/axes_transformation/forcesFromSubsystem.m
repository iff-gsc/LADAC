function [ F_Kb, M_Kb ] = forcesFromSubsystem(R_sub_b, euler_angles_sub, F_sub, M_sub)

%forcesFromSubsystem computes the forces and moments in the parent system
%
% Inputs:
%   R_sub_b             position vector with 3 elements of the subsystem
%                       in body-fixed system in [m]
%   euler_angles_sub    vector with 3 elements containing the Euler angles
%                       Phi,Theta and Psi angles between subsystem relative
%                       to the body-fixed system, notation in body-fixed
%                       system in [rad]
%
%   F_sub               forces in the rotated and shifted subsystem in [N]
%   M_sub               moments in the rotated and shifted subsystem in [N]
%
% Outputs:
%   F_Kb                resulting forces in body-fixed system in [N]
%   M_Kb                resulting moments in body-fixed system in [N]
%
%

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Fabian Guecker
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

F_Kb = euler2Dcm(euler_angles_sub)' * F_sub;

M_Kb = euler2Dcm(euler_angles_sub)' * M_sub + cross(R_sub_b, F_Kb) ;




