function [ n_g_des, Delta_q, Delta_r, q_des, r_des, a_abs_des, a_abs ] = ...
    indiPlaneAcc2LeanQR( nu_a_Kb_yz, V, M_bg, a_Kg, n_g_des_last, Delta_Phi_comp )
% indiPlaneAcc2LeanQR incremental inversion to convert desired acceleration
% to a desired lean vector as well as pitch rate and yaw rate
% 
% Syntax:
%   [ n_g_des, Delta_q, Delta_r, q_des, r_des, a_abs_des, a_abs ] = ...
%       indiPlaneAcc2LeanQR( nu_a_Kb_yz, V, M_bg, a_Kg )
%   [ n_g_des, Delta_q, Delta_r, q_des, r_des, a_abs_des, a_abs ] = ...
%       indiPlaneAcc2LeanQR( nu_a_Kb_yz, V, M_bg, a_Kg, n_g_des_last )
% 	[ n_g_des, Delta_q, Delta_r, q_des, r_des, a_abs_des, a_abs ] = ...
%       indiPlaneAcc2LeanQR( nu_a_Kb_yz, V, M_bg, a_Kg, n_g_des_last, ...
%                           Delta_Phi_comp )
% 
% Inputs:
%   nu_a_Kb_yz          Desired acceleration (without gravity) in body
%                       frame in lateral (y_b) and vertical (z_b)
%                       direction (2x1 array), in m/s^2
%   V                   Airspeed (scalar), in m/s
%   M_bg                Rotation matrix from geodetic frame (g) to body
%                       frame (b) (3x3 matrix)
%   a_Kg                Acceleration (without gravity) in geodetic frame
%                       (g) (3x1 array), in m/s^2
%   n_g_des_last        Last value of function output n_g_des
%   Delta_Phi_comp      To be compensated increment in roll angle,
%                       in rad
% 
% Outputs:
%   n_g_des             Desired lean vector (3x1 array), unit vector (see
%                       dcm2LeanVector)
%   Delta_q             Desired increment of pitch rate (scalar), in rad/s
%   Delta_r             Desired increment of yaw rate (scalar), in rad/s
%   q_des               Desired (non-incremental) pitch rate (scalar), in
%                       rad/s
%   r_des               Desired (non-incremental) yaw rate (scalar), in
%                       rad/s
%   a_des_abs           Desired absolute specific lift (scalar), in m/s^2
%   a_abs               Measured absolute specific lift (scalar), in m/s^2

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2026 Yannic Beyer
%   Copyright (C) 2026 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

if nargin < 6
    Delta_Phi_comp = zeros(1,class(nu_a_Kb_yz));
end
if nargin < 5
    n_g_des_last = zeros(3,1,class(M_bg));
end
n_g_des = zeros(3,1,class(nu_a_Kb_yz));

% Measured acceleration (without gravitation) in b frame
a_Kb = M_bg*a_Kg;
a_Kb_yz = a_Kb(2:3);

% Gravitational acceleration vector
% in g frame
g_g = [0;0;9.81];
% in b frame
g_b = M_bg*g_g;
g_b_yz = g_b(2:3);

% Desired specific lift vector
a_b_yz_des = nu_a_Kb_yz - g_b_yz;
% Desired specific lift
a_abs_des = norm(a_b_yz_des);
a_abs = norm(a_Kb_yz-g_b_yz);

% Avoid roll flip (apply negative lift instead)
n_b_des = divideFinite( [0;a_b_yz_des], a_abs_des );
if n_b_des(3) >= 0
    n_b_des(3) = -n_b_des(3);
    n_b_des(2) = -n_b_des(2);
end

if a_abs_des < 9.81/3
    if nargin < 5
        n_g_des_last(:) = dcm2LeanVector(M_bg);
    end
    n_g_des(:) = n_g_des_last;
else
    n_g_des(:) = M_bg'*n_b_des;
end

% 2D rotation matrix from y_b-z_b to future y_f-z_f frame due to expected
% increament in roll angle
% --> The f frame is the future b frame.
M_fb = [cos(Delta_Phi_comp),sin(Delta_Phi_comp);-sin(Delta_Phi_comp),cos(Delta_Phi_comp)];
nu_a_Kf_yz = M_fb*nu_a_Kb_yz;

% Estimated angular velocity vector in b frame (2D)
omega_i = divideFinite( a_Kb_yz, V );
q_i = -omega_i(2);
r_i = omega_i(1);

% Desired angular velocity vector in f frame (2D)
omega_des = divideFinite( nu_a_Kf_yz, V );
q_des = -omega_des(2);
r_des = omega_des(1);

% Incremental angular velocities in b frame
Delta_q = q_des - q_i;
Delta_r = r_des - r_i;

end