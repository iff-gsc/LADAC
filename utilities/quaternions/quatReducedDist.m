function [q_cmd_red, psi] = quatReducedDist(q_cmd, q_e)
% quatReducedDist split attitude in vertical axis alignment and yaw angle
%   [q_cmd_red, psi] = quatReducedDist(q_cmd, q_e) computes the reduced
%   attitude, of the commanded attitude q_cmd in respect to the current
%   attitude q_e. The result q_cmd_red is the commanded attitude that can
%   be reached from the current attitude q_e with a single rotation without
%   a rotation about the body fixed vertical axis. When the reduced
%   commanded attitude is reached, the missing rotation about the vertical
%   axis is given by psi in rad.
% 
% Syntax:
%   [q_cmd_red, psi] = quatReducedDist(q_cmd, q_e)
% 
% Inputs:
%   q_cmd           target attitude
%                   quaternion (4x1 array), dimensionless
%
%   q_e             current attitude
%                   quaternion (4x1 array), dimensionless
% 
% Outputs:
%   q_cmd           target attitude
%                   quaternion (4x1 array), dimensionless
%
%   psi             yaw angle
%                   (scalar) in rad
% 
% See also:
%   quatDistance, quatSlerp
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Fabian Guecker
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

M_bg_e = quat2Dcm(q_e);
vec_e = dcm2LeanVector( M_bg_e );

M_bg_cmd = quat2Dcm(q_cmd);
vec_cmd = dcm2LeanVector( M_bg_cmd );

psi = acos(min(max(dot(vec_e, vec_cmd),-1),1));

cross_q_e_q_cmd = cross(vec_e, vec_cmd);

q_e_red = [cos(psi/2); sin(psi/2) * cross_q_e_q_cmd / max(norm(cross_q_e_q_cmd), eps)];

q_cmd_red = quatMultiply(q_e_red, q_e);

end