function [ n_b, n_b_dt, n_b_dt2 ] = leanVectorDerivTrafo( n_g, n_g_dt, ...
    n_g_dt2, M_bg, omega_Kb, omega_Kb_dt )
% leanVectorDerivTrafo lean vector and time derivatives in other frame
%   The lean vector is a unit vector that points in -z_b direction or to
%   the desired -z_b direction. With this function the lean vector and its
%   first and second time derivative represented in frame g are transformed
%   to the representation in frame b.
% 
% Syntax:
%   [ n_b, n_b_dt, n_b_dt2 ] = leanVectorDerivTrafo( n_g, n_g_dt, ...
%                               n_g_dt2, M_bg, omega_Kb, omega_Kb_dt )
% 
% Inputs:
%   n_g             lean vector (3x1 array) represented in frame g,
%                   dimensionless
%   n_g_dt          time derivative of input n_g
%   n_g_dt2         second time derivative of input n_g
%   M_bg            rotation matrix (3x3 array) for transformation from
%                   frame g to frame b, dimensionless
%   omega_Kb        angular velocity vector (3x1 array) of frame b w.r.t.
%                   frame g represented in frame b, in rad/s
%   omega_Kb_dt     time derivative of input omega_Kb
% 
% Outputs:
%   n_b             lean vector (3x1 array) represented in frame b,
%                   dimensionless
%   n_b_dt          time derivative of output n_b
%   n_b_dt2         second time derivative of output n_b
% 
% Literature:
%   [1] Sun, S., Wang, X., Chu, Q., & de Visser, C. (2020). Incremental
%       nonlinear fault-tolerant control of a quadrotor with complete loss
%       of two opposing rotors. IEEE Transactions on Robotics, 37(1),
%       116-130.
% 
% See also:
%   dcm2LeanVector, leanAngles2leanVector

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2019-2022 First Author
%   Copyright (C) 2022 Second Author
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

n_b = M_bg*n_g;

% [1], eq. (7)
n_b_dt = -cross(omega_Kb,n_b) + M_bg*n_g_dt;

% time derivative of rotation matrix
M_bg_dt = dcmDerivFromOmega( M_bg', omega_Kb )';

% time derivative of [1], eq. (7)
n_b_dt2 = - cross(omega_Kb_dt,n_b) - cross(omega_Kb,n_b_dt) ...
    + M_bg_dt*n_g_dt + M_bg*n_g_dt2;

end
