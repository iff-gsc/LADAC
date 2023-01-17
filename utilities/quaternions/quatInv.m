function q_out = quatInv( q )
% quatInv inverse of a quaternion
%   q_out = quatInv( q ) computes the inverse of quaternion in the form
%   of [w; x; y; z]. With the scalar part w and vector parts x, y and z.
% 
% Syntax:
%   q_out = quatInv( q )
% 
% Inputs:
%   q               quaternion (4x1 array), dimensionless
% 
% Outputs:
%   q_out           inverse of input (4x1 array), dimensionless
%                    
% Example: 
%   q = [0.7071; 0.7071; 0; 0];
%   q_out = templateFunction(q)
% 
% See also:
%   quatMultiply
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Fabian Guecker
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% Conjugate
q_conj = [q(1); -q(2); -q(3); -q(4)];

% Normalize
q_out = q_conj ./ (max( eps, q'*q));

end