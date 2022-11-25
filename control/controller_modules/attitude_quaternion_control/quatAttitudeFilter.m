function [int_omega, omega] = quatAttitudeFilter...
    (q_bg, dot_q_bg, q_cmd, omega_Kb)
% quatAttitudeFilter calculate errors for second order attitude filter
%   This function calculates the errors for a second order attitude filter
%   for the attitude and the first derivative. Quaternions are in the form 
%   of [w; x; y; z]. With the scalar part w and vector parts x, y and z. 
% 
% Syntax:
% function [int_omega, omega] = quatAttitudeFilter...
%    (q_bg, dot_q_bg, q_cmd, omega_Kb, fastApproximation)
% 
% Inputs:
%   q_bg            attitude of the body-fixed system
%                   quaternion (4x1 array), dimensionless
%
%   dot_q_bg        1. time derivative of the aircrafts attitude
%                   this value is used in case of fastApprox == false
%                   (3x1 array), dimensionless
%
%   omega_Kb        angular velocity in body-fixed system
%                   this value is used in case of fastApprox == true
%                   (3x1 array), rad/s
%
%   q_cmd           commanded attitude 
%                   quaternion (4x1 array), dimensionless
% 
% Outputs:
%   int_omega       attitude error in the body-fixed system as axis-angle
%                   representation int_omega = (theta*axis_vector)
%                   (3x1 array), rad
%
%   omega           angular velocity error in body-fixed system
%                   (3x1 array), rad/s
% 
% See also:
%   quatAttitudeControllerFullState, quatAttitudeControllerRedState
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Fabian Guecker
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% Use the closest mapping of the two quaternions towards [1 0 0 0]'
% q_cmd = sign(q_cmd'*q_bg)*q_cmd;
% above fails if sign of dot product returns zero!!!
if (q_cmd'*q_bg) < 0
    q_cmd = -q_cmd;
end

% Calculate Distance between the two attitudes
int_omega = 2.0 * quatLogDivide(q_cmd, q_bg, 1);
int_omega = int_omega(2:4);

omega = -omega_Kb;

end