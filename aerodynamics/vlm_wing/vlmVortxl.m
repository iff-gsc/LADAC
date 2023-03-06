function uvw = vlmVortxl( p1, p2, pc, Gamma )
% Katz & Plotkin (2001), page 254-255

r0 = p2 - p1;
r1 = pc - p1;
r2 = pc - p2;

r0_r1 = dot( r0, r1 );
r0_r2 = dot( r0, r2 );

abs_r1 = norm( r1, 2 );
abs_r2 = norm( r2, 2 );

r1_x_r2 = cross( r1, r2 );

abs_r1_x_r2_sq = dot( r1_x_r2, r1_x_r2 );

K = Gamma / ( 4*pi * abs_r1_x_r2_sq ) * ( r0_r1/abs_r1 - r0_r2/abs_r2 );

uvw = K * r1_x_r2;

end