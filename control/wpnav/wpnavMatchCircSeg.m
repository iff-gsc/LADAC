function [p_match,t,d] = wpnavMatchCircSeg(circ_seg,p)

% project point p to circle plane
H = dot( p - circ_seg.center, circ_seg.n );
p0 = p - H*circ_seg.n;

r_p0 = p0 - circ_seg.center;
r_start = circ_seg.start - circ_seg.center;
r_p0_abs = norm( r_p0, 2 );
r_start_abs = norm( r_start, 2 );
num = dot( r_p0 , r_start );
denom = r_p0_abs*r_start_abs;
angle_match = acosReal( divideFinite( num, denom ) );

t = divideFinite( angle_match, circ_seg.angle );

p_match = wpnavCircSegGetPos( circ_seg, t );

d = norm( p_match - p, 2 );

% Especially in case of turn angles greater than 90 degree, it may happen
% that the given position p is matched to the "wrong segment" of the
% circle. In order to avoid this, the parameter t is set to 2 in this case.
R_center = circ_seg.center - circ_seg.wp;
R_center_norm = norm(R_center,2);
R_center_unit = divideFinite( R_center, R_center_norm );
R_p = p - circ_seg.wp;
D = dot( R_p, R_center_unit );
if D > R_center_norm
    t = 2;
end

end