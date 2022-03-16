function v_rot = axisAngle( v, axis, angle ) %#codegen
% axisAngle rotates a vector about an axis with an angle [1].
% 
% Inputs:
%   v       vectors to be rotated (3xN array for N vectors)
%   axis    axis to rotate about (3xN array); no need to be a unit vector
%   angle   angles to rotate about (1xN array), in rad
% 
% Outputs:
%   v_rot   rotated vectors (3xN array)
% 
% Literature:
%   [1] https://en.wikipedia.org/wiki/Axis�angle_representation
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

axis = axis ./ repmat( vecnorm( axis, 2, 1 ), 3, 1 );

cos_angle = cos(angle);

v_rot = repmat(cos_angle,3,1).*v + repmat(sin(angle),3,1).*cross(axis,v) ...
    + repmat( (1-cos_angle) .* dot(axis,v), 3,1) .* axis;

end