function n_g_des = indiCopterAcc2LeanVector( nu_s_g_dt2, s_g_dt2, M_bg, g )
% indiCopterAcc2LeanVector desired lean angle for copters with INDI
%   This function can be used as INDI interface between a position
%   controller cascade and an attitude controller cascade for multicopters.
% 
% Syntax:
%   n_g_des = indiCopterAcc2LeanVector( nu_s_g_dt2, s_g_dt2, M_bg, g )
% 
% Inputs:
%   nu_s_g_dt2      pseudo-control input from position controller cascade;
%                   commanded 2nd time-derivative of the position in
%                   geodetic (g) frame (3x1 array), in m/s^2
%   s_g_dt2         measured 2nd time-derivative of the position in
%                   geodetic (g) frame (3x1 array), in m/s^2
%   M_bg            rotation matrix from geodetic (g) frame to body-fixed
%                   (b) frame (3x3 array), dimensionless
%   g               gravitational acceleration (scalar), in m/s^2
% 
% Outputs:
%   n_g_des         desired lean vector as input for the attitude
%                   controller cascade in geodetic (g) frame; the lean
%                   vector is used for reduced attitude control, it points
%                   into the direction of the thrust vector and is a unit
%                   vector (3x1 array), see [1], section II.B.,
%                   dimensionless
% 
% Literature:
%   [1] Sun, S., Wang, X., Chu, Q., & de Visser, C. (2020). Incremental
%       nonlinear fault-tolerant control of a quadrotor with complete loss
%       of two opposing rotors. IEEE Transactions on Robotics, 37(1),
%       116-130.
% 
% See also:
% 	ndiCopterAcc2LeanVector, controlEffectiveness2G1G2

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% measured actual lean vector
n_g_mes = dcm2LeanVector( M_bg );

% desired lean angle increment
Delta_n_g_des = divideFinite( nu_s_g_dt2 - s_g_dt2, norm( s_g_dt2 - [0;0;g], 2 ) ); 

% increment desired lean angle and assure unit vector
n_g_des = n_g_mes + Delta_n_g_des;
n_g_des =  divideFinite( n_g_des, norm( n_g_des, 2 ) );

end