function a = wpnavCircSegGetAcc(circ_seg,t,V)

p = wpnavCircSegGetPos( circ_seg, t );
dir_vec_unit = circ_seg.center - p;
dir_vec_unit = divideFinite( dir_vec_unit, norm( dir_vec_unit, 2 ) );
a = divideFinite( V*V, circ_seg.r ) * dir_vec_unit;

end