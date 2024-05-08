function circ_seg = wpnavCircSeg( waypoints3x3, wp_radius )
% wpnavCircSeg create circle segment struct from 3 waypoints
% 
% Syntax:
%   circ_seg = wpnavCircSeg( waypoints3x3, wp_radius )
% 
% Inputs:
%   waypoints3x3        3x 3D waypoints (3x3 matrix), where each column is
%                       one waypoint in north-east-down coordinate system,
%                       in m
%   wp_radius           Waypoint radius (scalar), in m
% 
% Outputs:
%   circ_seg            Circle segment (struct as defined by this
%                       function), which connects the lines from waypoint
%                       1-->2 and waypoint 2-->3 with a circle segment (see
%                       waypoint_example)
% 
% See also:
%   wpnavMatchCircSeg, wpnavCircSegGetPos, wpnavCircSegGetVel

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2024 Yannic Beyer
%   Copyright (C) 2024 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

diff1 = waypoints3x3(:,2)-waypoints3x3(:,1);
diff2 = waypoints3x3(:,3)-waypoints3x3(:,2);
dist1 = norm(diff1,2);
dist2 = norm(diff2,2);
wp_rad = min([wp_radius,dist1/2,dist2/2]);

alpha = wrapAngle( acosReal( divideFinite( dot(diff1,diff2), dist1*dist2 ) ) );

if alpha < 100*eps(alpha)
    wp_rad(:) = 0;
end
d = 2*wp_rad*cos(alpha/2);
r = sqrtReal( divideFinite( (d/2)^2, (1-(cos(alpha/2))^2) ) );

n = cross( diff1, diff2 );
n = divideFinite( n, norm(n,2) );
if r > 100000
    r(:) = 100000;
    n(:) = [0;0;1];
end
diff1_unit = divideFinite( diff1, norm(diff1,2) );
diff2_unit = divideFinite( diff2, norm(diff2,2) );
r_vec = cross(n,diff1_unit);
r_vec = r * divideFinite( r_vec, norm(r_vec,2) );

seg_start = waypoints3x3(:,2) - wp_rad*diff1_unit;
seg_end = waypoints3x3(:,2) + wp_rad*diff2_unit;
        
center = waypoints3x3(:,2) - wp_rad*diff1_unit + r_vec;

% Radius, in m
circ_seg.r = r;
% 3D center in g frame, in m
circ_seg.center = center;
% 3D normal vector in g frame
circ_seg.n = n;
% Angle of circle segment, in rad
circ_seg.angle = alpha;
% Start position of circle segment in g frame, in m
circ_seg.start = seg_start;
% End position of circle segment in g frame, in m
circ_seg.end = seg_end;
% Resulting waypoint radius (wp_radius or smaller), in m
circ_seg.wp_rad = wp_rad;
% Waypoint in g frame, in m
circ_seg.wp = waypoints3x3(:,2);

end
