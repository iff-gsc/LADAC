% groundNormalForce computes normal forces depending on altitude, velocity
%   and parameters.
%   This function can be applied to multiple points. In that case
%   multiple altitudes and velocities must be given and as a result 
%   multiple normal forces are computed.
% 
% Literature:
%   -
% 
% Inputs:
%   alt_hit         altitude of hit points (1xn vector), in m
%   dot_alt_hit     time derivative of altitude of hit points (1xn vector),
%                   in m/s
%   k_spring        spring constant (scalar), in N/m
%   k_damp          damping coefficient (scalar), in N/(m/s)
%   soft_param      ground softness parameter (scalar) which reduces the 
%                   force from the spring model for small penetration of 
%                   the ground, in N
%   F_max           maximum applied normal force (scalar) i.e. due to
%                   irreversible deformations, in N
% 
% Outputs:
%   F_N             total normal force (1xn vector), in N
%   F_N_spring      normal force (1xn vector) due to spring model, in N
%   F_N_damp        normal force (1xn vector) due to damping model, in N
% 
% See also: frictionModel
% 

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

function [ F_N, F_N_spring, F_N_damp ] = groundNormalForce( alt_hit, ...
    dot_alt_hit, k_spring, k_damp, soft_param, F_max ) %#codegen

% only apply any force if point hits the ground
condition_1 = alt_hit < 0;
% compute normal force due to spring stiffness
F_N_spring_linear = - alt_hit * k_spring .* condition_1;
% reduce normal force for small e_z_g_hit ( f(x) = x^2 / ( x + k ) )
F_N_spring = F_N_spring_linear.^2 ./ ...
    ( F_N_spring_linear + soft_param );

% only apply damping force if point is moving towards the ground
% (avoid downwards oriented normal force due to damping)
condition_2 = dot_alt_hit < 0;
% compute normal force due to damping
F_N_damp = - dot_alt_hit * k_damp .* condition_1 .* condition_2;

% compute total normal force
F_N = min( F_N_spring + F_N_damp, F_max );

end