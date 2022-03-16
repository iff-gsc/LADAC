function q_out = quatExp( q )
% quatConj exponential of a quaternion
%   q_out = quatExp( q ) computes the exponential of quaternion in the form
%   of [w; x; y; z]. With the scalar part w and vector parts x, y and z.
% 
% Syntax:
%   q_out = quatExp( q )
% 
% Inputs:
%   q               quaternion (4x1 array), dimensionless
% 
% Outputs:
%   q_out           exponential of input (4x1 array), dimensionless
%                    
% Example: 
%   q = [0.7071; 0.7071; 0; 0];
%   q_out = quatExp(q)
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

qw = q(1);

qv = q(2:4);

norm_qv = max(norm(qv, 2), eps);

q_out = [cos(norm_qv); qv*sin(norm_qv)/norm_qv] * exp(qw);

end