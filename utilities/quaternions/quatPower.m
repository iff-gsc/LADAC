function [q_out] = quatPower(q, t)
% quatConj power t of a quaternion (q^t)
%   q_out = quatPower( q ) computes the power of t for the quaternion q
%   in the form of [w; x; y; z]. With the scalar part w and vector parts
%   x, y and z.
% 
% Syntax:
%   q_out = quatPower( q, t )
% 
% Inputs:
%   q               quaternion (4x1 array), dimensionless
%
%   t               exponent (scalar), dimensionless
% 
% Outputs:
%   q_out           power of t of input q (4x1 array), dimensionless
%                    
% Example: 
%   q = [0.7071; 0.7071; 0; 0];
%   q_out = quatPower([0.7071 0 0.7071 0]', 3)
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

q_out = quatExp(t * quatLog(q));

end

