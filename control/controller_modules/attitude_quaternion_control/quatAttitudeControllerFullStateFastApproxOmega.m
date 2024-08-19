function [int_omega, omega, dot_omega] = ...
    quatAttitudeControllerFullStateFastApproxOmega...
    (q_bg, omega_Kb, dot_omega_Kb, q_cmd, omega_cmd, dot_omega_cmd)
% quatAttitudeControllerFullStateFastApprox calculate errors for attitude 
% control. This is a fast approximation and should only be used, if the
% commanded attitude in very close to the current attitude of the aircraft
% all the time. Otherwise the commanded errors does not match the right
% axis on the aircraft and the controller become unstable. Suitable for 
% aircraft with low dynamics or simulations.  This version has frame-fixed
% angular rates and accelerations as input instead of the quaternion 
% derivatives
%   This function calculates the errors for full state attitude control for
%   the attitude and their first derivatives. Quaternions are in the form 
%   of [w; x; y; z]. With the scalar part w and vector parts x, y and z. 
% 
% Syntax:
% [int_omega, omega, dot_omega_err] = quatAttitudeControllerFullState...
%    (q_bg, omega_Kb, dot_omega_Kb, q_cmd, dot_q_cmd, ddot_q_cmd, dt)
% 
% Inputs:
%   q_bg            attitude of the body-fixed system
%                   quaternion (4x1 array), dimensionless
%
%   omega_Kb        angular velocity in body-fixed system
%                   (3x1 array), rad/s
%
%   dot_omega_Kb    angular acceleration in body-fixed system
%                   (3x1 array), rad/s^2
%
%   q_cmd           commanded attitude 
%                   quaternion (4x1 array), dimensionless
%
%   dot_q_cmd       1. time derivative of the commanded attitude
%                   quaternion (4x1 array), dimensionless
%
%   ddot_q_cmd      2. time derivative of the commanded attitude
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
%   dot_omega       angular acceleration error in body-fixed system
%                   (3x1 array), rad/s
% 
% See also:
%   quatAttitudeControllerFullState
% 

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Fabian Guecker
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% q_cmd = sign(q_cmd'*q_bg)*q_cmd;
% above fails if sign of dot product returns zero!!!
if (q_cmd'*q_bg) < 0
    q_cmd = -q_cmd;
end


%% Integral Omega Part
int_omega_q = 2.0 * quatLogDivide(q_cmd, q_bg, 1);
int_omega   = int_omega_q(2:4);

%% Velocity Part
omega = omega_cmd - omega_Kb;

%% Acceleration Part
dot_omega = dot_omega_cmd - dot_omega_Kb;

end
