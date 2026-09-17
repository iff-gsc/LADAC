function [ nu_a_Kb_yz_lim, f_b_yz_des_lim, f_max, f_des_abs, f_des_abs_lim ] = ...
    limitAccByLift( nu_a_Kb_yz, V, rho, M_bg, cl_lim ) %#codegen
% limitAccByLift limits the desired acceleration based on the available
% aerodynamic lift
%
% The desired specific-force vector is obtained by subtracting gravity
% from the desired kinematic acceleration. Its magnitude is limited using
% the applicable positive or negative lift-coefficient limit while its
% direction is retained. The limited kinematic acceleration is then
% reconstructed by adding gravity.
%
% Syntax:
%   [nu_a_Kb_yz_lim, f_max, f_des_abs, f_des_abs_lim] = ...
%       limitAccByLift(nu_a_Kb_yz, V, rho, M_bg, lift_lim)
%
% Inputs:
%   nu_a_Kb_yz        Desired kinematic acceleration in body frame in
%                     lateral (y_b) and vertical (z_b) direction
%                     (2x1 array), in m/s^2
%   V                 Airspeed (scalar), in m/s
%   rho               Air density (scalar), in kg/m^3
%   M_bg              Rotation matrix from geodetic frame (g) to body
%                     frame (b) (3x3 matrix)
%   cl_lim            Structure containing the parameters used for lift
%                     limitation, with the following fields:
%       cl_lim.use      Enable lift limitation (logical scalar)
%       cl_lim.S        Wing reference area, in m^2
%       cl_lim.m        Aircraft mass, in kg
%       cl_lim.cl_max   Maximum positive lift coefficient
%       cl_lim.cl_min   Minimum negative lift coefficient
%
% Outputs:
%   nu_a_Kb_yz_lim    Lift-limited desired kinematic acceleration in body
%                     frame in lateral (y_b) and vertical (z_b) direction
%                     (2x1 array), in m/s^2
%   f_max             Maximum available specific-lift magnitude for the
%                     applicable lift direction (scalar), in m/s^2
%   f_des_abs         Magnitude of the unconstrained desired specific-lift
%                     vector (scalar), in m/s^2
%   f_des_abs_lim     Magnitude of the lift-limited desired specific-lift
%                     vector (scalar), in m/s^2

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2026 Yannic Beyer
%   Copyright (C) 2026 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

g_b = M_bg * [0; 0; 9.81];

f_b_yz_des  = nu_a_Kb_yz - g_b(2:3);
f_des_abs   = norm(f_b_yz_des);

if f_b_yz_des(2) <= 0
    cl_max = cl_lim.cl_max;
else
    cl_max = abs(cl_lim.cl_min);
end

f_max = rho/2 * V^2 * cl_lim.S/cl_lim.m * cl_max;

if cl_lim.use
    scale = min(1, divideFinite(f_max, f_des_abs));
else
    scale = 1;
end

f_b_yz_des_lim  = scale * f_b_yz_des;

f_des_abs_lim   = norm(f_b_yz_des_lim);
nu_a_Kb_yz_lim  = f_b_yz_des_lim + g_b(2:3);

end