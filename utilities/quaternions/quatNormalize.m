function q_out = quatNormalize( q )
% quatNormalize normalize a quaternion to unit length
%   q_out = quatNormalize( q ) normalize a quaternion in the form of 
%   [w; x; y; z]. With the scalar part w and vector parts x, y and z.
% 
% Syntax:
%   q_out = quatNormalize( q )
% 
% Inputs:
%   q               quaternion (4x1 array), dimensionless
% 
% Outputs:
%   q_out           normalized input (4x1 array), dimensionless
%                    
% Example: 
%   q = [0.7071; 0.7071; 0; 0];
%   q_out = quatNormalize(q)
% 
% See also:
%   quatMultiply, quatInv
% 

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Fabian Guecker
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% Normalize
q_out = q / max( eps, norm(q, 2) );

end