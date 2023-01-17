function Omega_Kb = leanVectorDeriv2Omega( n_g_dt, M_bg )
% leanVectorDeriv2Omega angular rates from lean vector time derivative
% 
% Syntax:
%   Omega_Kb = leanVectorDeriv2Omega( n_g_dt, M_bg )
% 
% Inputs:
%   n_g_dt          time derivative of lean vector (3x1 array), see
%                   dcm2LeanVector, in 1/s
%   M_bg            direction cosine matrix (3x3 array) from NED
%                   frame (g) to body frame (b)
% 
% Outputs:
%   Omega_Kb        angular velocity vector of body frame b w.r.t. NED
%                   frame g represented in frame b, in rad/s

% See also:
%   dcm2LeanVector, leanVectorDerivTrafo
% 
% Literature:
%   [1] Sun, S., Wang, X., Chu, Q., & de Visser, C. (2020). Incremental
%       nonlinear fault-tolerant control of a quadrotor with complete loss
%       of two opposing rotors. IEEE Transactions on Robotics, 37(1),
%       116-130. url: https://arxiv.org/pdf/2002.07837.pdf

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% from [1], Eq. (7) with n_b_d = [0;0;-1] and n_b_d_dt = [0;0;0]
cross_Omega_Kb_n_b = M_bg*n_g_dt;

Omega_Kb = [ cross_Omega_Kb_n_b(2); -cross_Omega_Kb_n_b(1); 0 ];

end