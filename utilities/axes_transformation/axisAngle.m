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
%   [1] https://en.wikipedia.org/wiki/Axis-angle_representation
% 

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

v_rot       = zeros( size(v), class(v) );
axis_length = vecnorm( axis, 2, 1 );
for i = 1:3
    axis(i,:) = divideFinite( axis(i,:), axis_length );
end
cos_angle   = cos(angle);
cos_angle_1 = 1-cos_angle;
sin_angle   = sin(angle);
cross_prod  = crossFast(axis,v);
dot_prod    = dot(axis,v);
for i = 1:3
    v_rot(i,:) = cos_angle.*v(i,:) + sin_angle.*cross_prod(i,:) ...
        + cos_angle_1 .* dot_prod .* axis(i,:);
end