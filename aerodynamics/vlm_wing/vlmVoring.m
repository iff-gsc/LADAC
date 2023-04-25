function uvw = vlmVoring( p1, p2, p3, p4, pc, Gamma )
% Katz & Plotkin (2001), page 255-256

uvw_1 = vlmVortxl( p1, p2, pc, Gamma );
uvw_2 = vlmVortxl( p2, p3, pc, Gamma );
uvw_3 = vlmVortxl( p3, p4, pc, Gamma );
uvw_4 = vlmVortxl( p4, p1, pc, Gamma );

uvw = uvw_1 + uvw_2 + uvw_3 + uvw_4;

end