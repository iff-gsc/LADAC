function q_norm = quatNorm( q )
% quatConj norm of a quaternion
%   q_out = quatConj( q ) computes the norm of quaternion in the form
%   of [w; x; y; z]. With the scalar part w and vector parts x, y and z.
% 
% Syntax:
%   q_out = quatNorm( q )
% 
% Inputs:
%   q               quaternion (4x1 array), dimensionless
% 
% Outputs:
%   q_norm          norm of input (scalar), dimensionless
%                    
% Example: 
%   q = [0.7071; 0.7071; 0; 0];
%   q_out = quatNorm(q)
% 
% See also:
%   quatMultiply, quatInv, quatNorm
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Fabian Guecker
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% Norm of quaternion
q_norm = norm(q, 2);

end