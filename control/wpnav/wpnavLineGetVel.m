function v = wpnavLineGetVel(p1,p2,V)

dir_vec_unit = p2-p1;
dir_vec_unit = divideFinite( dir_vec_unit, norm( dir_vec_unit, 2 ) );
v = V * dir_vec_unit;

end