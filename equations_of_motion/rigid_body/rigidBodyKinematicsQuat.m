% rigidBodyKinematicsQuat computes the time derivative of quaternions and
% position.
%   The times derivatives are computed depending on the rotational
%   velocity, the velocity, the quaternion of the rigid body as well as the
%   DCM of the velocity frame to the position frame.
% 
% Literature:
%   [1] Stevens, B. L. et al. (2016): Aircraft Control and Simulation.
%       Dynamics, Control Design, and Autonomous Systems. 3rd ed. Wiley.
%   [2] The MathWorks, Inc. (2002): Aerospace Blockset. For Use with 
%       Simulink. User's Guide. Version 1.
% 
% Inputs:
%   omega_Kb        three dimensional angular velocity vector of the rigid
%                   body relative to the earth represented in body-fixed
%                   frame, in rad/s
%   V_Kb            three dimensional velocity vector of the rigid body
%                   relative to the earth represented in body-fixed frame,
%                   in m/s
%   q_bg            four dimensional quaternion vector of the rigid body
%                   relativ to the earth, in 1
%   M_bg            rotation 3x3 matrix (DCM) from the earth frame (g) to 
%                   the body-fixed frame (g), in 1   
% 
% Outputs:
%   dot_q_bg        four dimensional vector of the quaternion time
%                   derivative, in 1/s
%   dot_s_g         three dimensional time derivative of the position 
%                   vector in earth frame, in m/s
% 
% See also: rigidBodyKinetics
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

function [ dot_q_bg, dot_s_g ] = ...
    rigidBodyKinematicsQuat( omega_Kb, V_Kb, q_bg, M_bg )%#codegen

% compute the omega cross product matrix according to [1, page 8]
omega_cross_product = [ 0, omega_Kb(3), -omega_Kb(2); ...
                        -omega_Kb(3), 0, omega_Kb(1); ...
                        omega_Kb(2), -omega_Kb(1), 0 ];

% normalize quaternion
q_bg_norm = quatNormalize( q_bg );

% compute the time derivative of the quaternions vector according to
% [1, page 51]
dot_q_bg = 0.5 * [ 0, -omega_Kb';...
                   omega_Kb, omega_cross_product ] ...
            * q_bg_norm;

% high gain quaternion normalization according to [2, page 3-60f]
k_quat = 1;
q_error = 1 - dot( q_bg, q_bg ); 
dot_q_bg = dot_q_bg + q_error * k_quat * q_bg_norm;

% computate of the time derivative of the position vector in g frame
% according to [1, page 41 or 42]
dot_s_g = M_bg' * V_Kb;