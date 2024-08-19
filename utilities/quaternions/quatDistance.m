function q_out = quatDistance(q, qr)
% quatDistance calculate the distance between two quaternions
%   q_out = quatDistance(q, qr, shortest_path) computes a optimized 
%   quatLog(quatDivide(q, qr)) without calculating the scalar part.
%   No normalisation is performed. The function calculates always the
%   shortest path between the two quaternions, that is maximum 180 deg.
%   The return value is a rotation vector with the distance in rad for each
%   axis for a constant angular velocity. To get the total distance
%   calculate the norm of the result.
%   Quaternions has to be in the form of [w; x; y; z]. With the scalar part
%   w and vector parts x, y and z. 
% 
% Syntax:
%   q_out = quatDistance(q, qr, shortest_path)
% 
% Inputs:
%   q               first quaternion (4x1 array), dimensionless
%   qr              second quaternion (4x1 array), dimensionless
% 
% Outputs:
%   q_out           pure rotation vector (4x1 array), radian
%                   [0; x_dist, y_dist, z_dist]
%                    
% Example: 
%   q  = [1; 0; 0; 0];
%   qr = [-0.7071; 0.7071; 0; 0];
%   q_out = quatDistance(q, qr)*360/pi
% 
% See also:
%   quatMultiply, quatInv, quatNorm
% 

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Fabian Guecker
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% Invert quaternion qr
qr(2:4) = -qr(2:4);

% Vector part of quaternion product
qv = [qr(1)*q(2); qr(1)*q(3); qr(1)*q(4)] + ...
           [q(1)*qr(2); q(1)*qr(3); q(1)*qr(4)] +...
           [qr(3)*q(4) - qr(4)*q(3); ...
            qr(4)*q(2) - qr(2)*q(4); ...
            qr(2)*q(3) - qr(3)*q(2)];

% Scalar part of quaternion product
qw = qr(1)*q(1) - qr(2)*q(2) - qr(3)*q(3) - qr(4)*q(4);

% Prevent division by zero
norm_qv = max(norm(qv), eps);

% Calculate the shortest angular distance for each axis
q_out = -2.0 * (qv/norm_qv) * atan2( norm_qv, abs(qw)) * sign(qw);

end

