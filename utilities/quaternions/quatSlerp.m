function [q_out] = quatSlerp(p, q, t)
% quatSlerp calculates spherical linear interpolation
%   q_out = quatMultiply( q ) computes spherical linear interpolation of
%   two quaternions in the form of [w; x; y; z]. With the scalar part w and
%   vector parts x, y and z. The scalar parameter t is the interpolation
%   coefficient in the range [0, 1].
% 
% Syntax:
%   q_out = quatMultiply( p, q, t )
% 
% Inputs:
%   p               first quaternion (4x1 array), dimensionless
%   q               second quaternion (4x1 array), dimensionless
%   t               interpolation coefficient (scalar), dimensionless
% 
% Outputs:
%   q_out           result of slerp  (4x1 array), dimensionless
%                    
% Example: 
%   p = [0.7071; 0.7071; 0; 0];
%   q = [0.7071; -0.7071; 0; 0];
%   t = 0.5;
%   q_out = quatMultiply(q, qr, t)
% 
% See also:
%   quatDivide, quatInv, quatNorm
% 

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Fabian Guecker
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

q_out =  quatMultiply(quatPower(quatMultiply(q, quatInv(p)), t), p);

end

