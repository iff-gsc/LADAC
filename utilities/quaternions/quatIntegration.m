function q_out = quatIntegration(q0, omega, dot_omega, dt)
% quatIntegration integrate angular velocity and angular acceleration
% for given time step dt to the inital quaternion q0.
%   q_out = quatIntegration(q0, omega, dot_omega, dt) integrates the
%   angular velocity exactly and the angular acceleration with a first
%   order approximation to the intal quaternions in the form of
%   [w; x; y; z]. With the scalar part w and vector parts x, y and z.
% 
% Syntax:
%   q_out = quatIntegration(q0, omega, dot_omega, dt)
% 
% Inputs:
%   q0              inital quaternion (4x1 array), dimensionless
%   omega           angular velocity vector (3x1 array), rad/sec
%   dot_omega       angular acceleration vector (3x1 array), rad/sec
%   dt              integration time (3x1 array), seconds
% 
% Outputs:
%   q_out           quaternion after integration (4x1 array), dimensionless
%                    
% Example: 
%   q0    = [1; 0; 0; 0];
%   omega = [pi; 0; 0];
%   dot_omega = [0; 0; 0]
%   dt = 0.5;
%   q_out = quatIntegration(q0, omega, dot_omega, dt);
% 
% See also:
%   quatMultiply, quatInv, quatNorm
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Fabian Guecker
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% Exact analytical integration
omega_bar = omega + 1/2*dot_omega * dt;

% Prevent division by zero
norm_omega_bar = max(norm(omega_bar), eps);
q_exp = [cos(norm_omega_bar*dt/2);...
    omega_bar/norm_omega_bar * sin(norm_omega_bar*dt/2)]';

% vector part
vec = [q0(1)*q_exp(2); q0(1)*q_exp(3); q0(1)*q_exp(4)] + ...
    [q_exp(1)*q0(2); q_exp(1)*q0(3); q_exp(1)*q0(4)] +...
    [q0(3)*q_exp(4) - q0(4)*q_exp(3); ...
    q0(4)*q_exp(2) - q0(2)*q_exp(4); ...
    q0(2)*q_exp(3) - q0(3)*q_exp(2)];

% scalar part
scalar = q0(1)*q_exp(1) - q0(2)*q_exp(2) - q0(3)*q_exp(3) - q0(4)*q_exp(4);

% return quaternion
q_out = [scalar; vec];

end
