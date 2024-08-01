function [ Delta_Phi, Delta_q, Delta_r, a_des_abs, a_abs ] = ...
    indiPlaneAcc2PhiQR( nu_a_Kb_yz, V, M_bg, a_Kg, Delta_Phi_f )
% indiPlaneAcc2PhiQR incremental inversion to convert desired acceleration
% to a desired roll angle as well as pitch rate and yaw rate
% 
% Syntax:
% 	[ Delta_Phi, Delta_q, Delta_r, a_des_abs, a_abs ] = ...
%       indiPlaneAcc2PhiQR( nu_a_Kb_yz, V, M_bg, a_Kg, Delta_Phi_f )
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
%   Delta_Phi_f         Filtered desired increment of roll angle (scalar),
%                       in rad
% 
% Outputs:
%   Delta_Phi           Desired increment of roll angle (scalar), in rad
%   Delta_q             Desired increment of pitch rate (scalar), in rad/s
%   Delta_r             Desired increment of yaw rate (scalar), in rad/s
%   a_des_abs           Desired absolute specific lift (scalar), in m/s^2
%   a_abs               Measured absolute specific lift (scalar), in m/s^2

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2024 Yannic Beyer
%   Copyright (C) 2024 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

method = 1;

% Measured acceleration (without gravitation) in b frame
a_Kb = M_bg*a_Kg;
a_Kb_yz = a_Kb(2:3);

% Gravitational acceleration vector
% in g frame
g_g = [0;0;9.81];
% in b frame
g_b = M_bg*g_g;
g_b_yz = g_b(2:3);

a_g = a_Kg - g_g;

% Estimated specific lift vector
a_b = M_bg*a_g;
a_b_yz = a_b(2:3);

% Estimated specific lift
a_abs = norm(a_b_yz);

% Desired specific lift vector
a_b_yz_des = nu_a_Kb_yz - g_b_yz;
% Desired specific lift
a_des_abs = norm(a_b_yz_des);

% 2 triangles
if method == 1
    % Gravitational acceleration in y_b-z_b plane
    g_abs = norm(g_b_yz);
    % Desired acceleration (without gravity) in y_b-z_b plane
    nu_abs = norm(nu_a_Kb_yz);
    % Estimated acceleration (wihtout gravity) in y_b-z_b plane
    a_K_abs = norm(a_Kb_yz);

    % Estimated bank angle from law of cosines
    Phi_i = acosReal( divideFinite( g_abs^2 + a_abs^2 - a_K_abs^2, 2*g_abs*a_abs ) );
    % Sign of estimated bank angle
    forward = M_bg'*[1;0;0];
    cross_i_g = cross(forward,a_Kg);
    Phi_i_sign = sign(cross_i_g(3));
    Phi_i = Phi_i_sign*Phi_i;
    
    % Desired bank angle from law of cosines
    Phi_des = acosReal( divideFinite( g_abs^2 + a_des_abs^2 - nu_abs^2, 2*g_abs*a_des_abs ) );
    % Sign of desired bank angle
    cross_des_g = cross([0;g_b_yz],[0;nu_a_Kb_yz]);
    Phi_des_sign = -sign(cross_des_g(1));
    Phi_des = Phi_des_sign*Phi_des;
    
    % Incremental desired bank angle
    Delta_Phi = Phi_des - Phi_i;

% 1 triangle
else
    
    % Incremental desired acceleration vector in y_b-z_b plane
    Delta_a_b_yz = a_b_yz_des - a_b_yz;
    Delta_a_abs = norm(Delta_a_b_yz);
    
    % Incremental desired bank angle from law of cosines
    Delta_Phi = acosReal( divideFinite( a_abs^2 + a_des_abs^2 - Delta_a_abs^2, 2*a_abs*a_des_abs ) );
    % Sign of incremental desired bank angle
    c = cross( [0;a_b_yz], [0;a_b_yz_des] );
    Delta_Phi_sign = sign(c(1));
    Delta_Phi = Delta_Phi_sign * Delta_Phi;

end

% 2D rotation matrix from y_b-z_b to filtered desired bank angle (y_f-z_f
% frame)
% --> The f frame is the future b frame.
M_fb = [cos(Delta_Phi_f),sin(Delta_Phi_f);-sin(Delta_Phi_f),cos(Delta_Phi_f)];

% Desired acceleration (without gravity) in y_f-z_f frame
nu_a_Kf_yz = M_fb*nu_a_Kb_yz;

if method == 1
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
    
else
    % Desired incremental acceleration in b frame (2D)
    Delta_nu = nu_a_Kf_yz - a_Kb_yz;
    % Desired incremental angular velocity in b frame (2D)
    Delta_omega = divideFinite( Delta_nu, V );
    Delta_q = -Delta_omega(2);
    Delta_r = Delta_omega(1);
end

end