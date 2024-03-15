function v = wpnavCircSegGetVel(circ_seg,t,V)

tangent_vec_p1_unit = cross( circ_seg.n, circ_seg.start-circ_seg.center );
tangent_vec_p1_unit = divideFinite( tangent_vec_p1_unit, norm( tangent_vec_p1_unit, 2 ) );

angle = t * circ_seg.angle;
tangent_vec_t_unit = axisAngle(tangent_vec_p1_unit,circ_seg.n,angle);

v = tangent_vec_t_unit * V;

end