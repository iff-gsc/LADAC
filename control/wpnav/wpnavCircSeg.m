function circ_seg = wpnavCircSeg( waypoints3x3, wp_radius )

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

% Radius
circ_seg.r = r;
% 3D center
circ_seg.center = center;
% 3D normal vector
circ_seg.n = n;
% Angle of circle segment, rad
circ_seg.angle = alpha;
% Start position of circle segment
circ_seg.start = seg_start;
% End position of circle segment
circ_seg.end = seg_end;
% Resulting waypoint radius (wp_radius or smaller)
circ_seg.wp_rad = wp_rad;
% Waypoint
circ_seg.wp = waypoints3x3(:,2);

end
