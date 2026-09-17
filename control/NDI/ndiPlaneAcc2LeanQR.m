function [ n_g_des, q_des, r_des, nu_a_Kb_yz_lim, f_max, f_abs_des, f_abs_des_lim ] = ...
    ndiPlaneAcc2LeanQR( nu_a_Kb_yz, V, M_bg, a_Kg, n_g_des_last, ...
    Delta_Phi_comp, rho, par )
% ndiPlaneAcc2LeanQR nonlinear dynamic inversion to convert desired
% acceleration to a desired lean vector as well as pitch rate and yaw rate
% 
% Syntax:
%   [ n_g_des, q_des, r_des, nu_a_Kb_yz_lim, f_max, f_abs_des, ...
%       f_abs_des_lim ] = ...
%       indiPlaneAcc2LeanQR( nu_a_Kb_yz, V, M_bg, a_Kg, n_g_des_last, ...
%       Delta_Phi_comp, rho, par )
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
%   rho                 Air density (scalar), in kg/m^3
%   par                 Structure containing the parameters with the
%                       following fields:
%       par.T_acc           Time constant for acceleration-based airspeed
%                           prediction (needed to avoid oscillation during
%                           stall prevention), in s
%       par.rll_acc_min     Minimum absolute desired specific force for
%                           activating roll control (below this threshold,
%                           the previous roll angle is retained), in m/s^2
%       par.cl_lim.use      Enable lift limitation (logical scalar)
%       par.cl_lim.S        Wing reference area, in m^2
%       par.cl_lim.m        Aircraft mass, in kg
%       par.cl_lim.cl_max   Maximum positive lift coefficient
%       par.cl_lim.cl_min   Minimum negative lift coefficient
% 
% Outputs:
%   n_g_des             Desired lean vector (3x1 array), unit vector (see
%                       dcm2LeanVector)
%   q_des               Desired (non-incremental) pitch rate (scalar), in
%                       rad/s
%   r_des               Desired (non-incremental) yaw rate (scalar), in
%                       rad/s
%   nu_a_Kb_yz_lim      Lift-limited desired kinematic acceleration in body
%                       frame in lateral (y_b) and vertical (z_b) direction
%                       (2x1 array), in m/s^2
%   f_max               Maximum available specific-lift magnitude for the
%                       applicable lift direction (scalar), in m/s^2
%   f_abs_des           Magnitude of the unconstrained desired
%                       specific-lift vector (scalar), in m/s^2
%   f_abs_des_lim       Magnitude of the lift-limited desired specific-lift
%                       vector (scalar), in m/s^2

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2026 Yannic Beyer
%   Copyright (C) 2026 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% Init
n_g_des = zeros(3,1,class(nu_a_Kb_yz));

% Stall prevention
a_Kb = M_bg * a_Kg;
V = V + a_Kb(1) * par.T_acc;
[ nu_a_Kb_yz_lim, f_b_yz_des_lim, f_max, f_abs_des, f_abs_des_lim ] = ...
    limitAccByLift( nu_a_Kb_yz, V, rho, M_bg, par.cl_lim );

% Avoid roll flip (apply negative lift instead)
n_b_des = divideFinite( [0;f_b_yz_des_lim], f_abs_des_lim );
if n_b_des(3) >= 0
    n_b_des(3) = -n_b_des(3);
    n_b_des(2) = -n_b_des(2);
end

% Avoid roll flip (apply negative lift instead)
if f_abs_des_lim < par.rll_acc_min
    n_g_des(:) = n_g_des_last;
else
    n_g_des(:) = M_bg'*n_b_des;
end

% 2D rotation matrix from y_b-z_b to future y_f-z_f frame due to expected
% increament in roll angle
% --> The f frame is the future b frame.
M_fb = [cos(Delta_Phi_comp),sin(Delta_Phi_comp);-sin(Delta_Phi_comp),cos(Delta_Phi_comp)];
nu_a_Kf_yz = M_fb*nu_a_Kb_yz_lim;

% Desired angular velocity vector in f frame (2D)
omega_des = divideFinite( nu_a_Kf_yz, V );
q_des = -omega_des(2);
r_des = omega_des(1);

end