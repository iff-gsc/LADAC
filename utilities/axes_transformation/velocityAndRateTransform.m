function [ V_sub, omega_sub ] = velocityAndRateTransform(V_Kb, omega_Kb, R_sub_b, M_xy)

%velocityAndRateTransform computes the velocity on a given Point due to
%angular velocity and translational velocity of of the body-fixed system
%
% Inputs:
%   R_sub_b             position vector of the point P with 3 elements of 
%                       the subsystem in body-fixed system in [m]
%   euler_angles_sub    vector with 3 elements containing the Euler angles
%                       Phi,Theta and Psi angles between subsystem relative
%                       to the body-fixed system, notation in body-fixed
%                       system in [rad]
%
%   V_Kb                velocity of the body-fixed system in [m/s]
%   omega_Kb            angular velocity in the body-fixed [rad/s]
%
% Outputs:
%   V_sub               resulting velocity in the subsystem [m/s]
%   omega_sub           resulting angular velocity in the subsystem [rad/s]
%

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Fabian Guecker
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

V_sub     = M_xy * ( V_Kb + cross(omega_Kb, R_sub_b) );

omega_sub = M_xy * omega_Kb;

end