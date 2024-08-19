function q_out = quatLog( q )
% quatConj logarithm of a quaternion
%   q_out = quatLog( q ) computes the logarithm of quaternion in the form
%   of [w; x; y; z]. With the scalar part w and vector parts x, y and z.
% 
% Syntax:
%   q_out = quatLog( q )
% 
% Inputs:
%   q               quaternion (4x1 array), dimensionless
% 
% Outputs:
%   q_out           logarithm of input (4x1 array), dimensionless
%                    
% Example: 
%   q = [0.7071; 0.7071; 0; 0];
%   q_out = quatLog(q)
% 
% See also:
%   quatMultiply, quatInv, quatNorm
% 

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Fabian Guecker
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************v

qw = q(1);

qv = q(2:4);

norm_qv = max(norm(qv, 2), eps);

q_out = [log(norm(q)); (qv/norm_qv) * atan2( norm_qv, qw)];

end