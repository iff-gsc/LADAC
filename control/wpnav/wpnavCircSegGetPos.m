function p = wpnavCircSegGetPos( circ_seg, t )

angle = t * circ_seg.angle;
p = circ_seg.center + axisAngle(circ_seg.start-circ_seg.center,circ_seg.n,angle);

end