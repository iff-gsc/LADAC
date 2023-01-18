% rigidBodyKinetics computes the time derivative of the velocity and
% angular velocity.
%   The times derivatives are computed depending on the force an moment
%   vector represented in body-fixed frame, mass and inertia matrix of the
%   body, the gravitational acceleration, the DCM from earth frame to
%   body-fixed frame as well as the velocity and the angular velocity.
% 
% Literature:
% [1]   Stevens, B. L. et al. (2016): Aircraft Control and Simulation.
%       Dynamics, Control Design, and Autonomous Systems. 3rd ed. Wiley.
% 
% Inputs:
%   R_b             three dimensional force vector represented in
%                   body-fixed frame, in N
%   Q_b             three dimensional moment vector represented in
%                   body-fixed frame, in N.m
%   m               scalar mass of the body, in kg
%   I_b             inertia 3x3 matrix in body-fixed frame, in kg.m^2
%   g               scalar gravitational acceleration, in m/s^2
%   M_bg            rotation 3x3 matrix (DCM) from the earth frame (g) to 
%                   the body-fixed frame (g), in 1         
%   omega_Kb        three dimensional angular velocity vector of the rigid
%                   body relative to the earth represented in body-fixed
%                   frame, in rad/s
%   V_Kb            three dimensional velocity vector of the rigid body
%                   relative to the earth represented in body-fixed frame,
%                   in m/s
% 
% Outputs:
%   omega_Kb_dt     three dimensional time derivative of the angular
%                   velocity vector of the rigid body relative to the earth
%                   represented in body-fixed frame, in rad/s^2
%   V_Kb_dt         three dimensional time derivative of the velocity
%                   vector of the rigid body relative to the earth
%                   represented in body-fixed frame, in m/^2
% 
% See also: rigidBodyKinematicsQuat
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

function [omega_Kb_dt, V_Kb_dt] = ...
    rigidBodyKinetics( R_b, Q_b, m, I_b, g, M_bg, omega_Kb, V_Kb )%#codegen

% compute the time derivative of the angular velocity vector according to
% [1, page 36 or 42]
omega_Kb_dt = I_b \ (Q_b - cross( omega_Kb, I_b * omega_Kb ) );

% compute the time derivative of the velocity vector according to [1, page
% 41 or 42]
V_Kb_dt = 1/m * R_b + M_bg * [0; 0; g] - cross( omega_Kb, V_Kb );