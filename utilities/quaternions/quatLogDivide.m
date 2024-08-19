function q_out = quatLogDivide(q, qr, shortest_path)
% quatLogDivide calculate the distance between two quaternions
%   q_out = quatLogDivide(q, qr, shortest_path) computes a optimized 
%   quatLog(quatDivide(q, qr)) without calculating the scalar part, as
%   result a pure quaternion is returned. No normalisation is performed.
%   It can be choosen between the true distance of the two quaternions or
%   the shortest path that is maximum 180 deg large.
%   The return value is a rotation vector with the distance in rad for each
%   axis for a constant angular velocity. To get the total distance
%   calculate the norm of the result.
%   Quaternions has to be in the form of [w; x; y; z]. With the scalar part
%   w and vector parts x, y and z. 
% 
% Syntax:
%   q_out = quatLogDivide(q, qr, shortest_path)
% 
% Inputs:
%   q               first quaternion (4x1 array), dimensionless
%   qr              second quaternion (4x1 array), dimensionless
%   shortest_path   boolean (scalar), dimensionless
% 
% Outputs:
%   q_out           pure rotation vector (4x1 array), radian
%                   [0; x_dist, y_dist, z_dist]
%                    
% Example: 
%   q  = [1; 0; 0; 0];
%   qr = [-0.7071; 0.7071; 0; 0];
%   dist_long = quatLogDivide(q, qr, false)*360/pi
%   dist_short = quatLogDivide(q, qr, true)*360/pi
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

% Decide if result contains real distance between the two quaternion
% or the shortest way between them, that is always >=180 deg.
if(shortest_path)
   qw = abs(qw); 
end

q_out = [0; (qv/norm_qv)] * atan2( norm_qv, qw);

end

