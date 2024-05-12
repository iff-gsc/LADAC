function y  = ardupilotJson(time,Omega_Kb,accel_b,s_g,q_bg,V_Kg)

ystr = sprintf('\n{"timestamp":%f,"imu":{"gyro":[%f,%f,%f],"accel_body":[%f,%f,%f]},"position":[%f,%f,%f],"quaternion":[%f,%f,%f,%f],"velocity":[%f,%f,%f]}\n',time,Omega_Kb(1),Omega_Kb(2),Omega_Kb(3),accel_b(1),accel_b(2),accel_b(3),s_g(1),s_g(2),s_g(3),q_bg(1),q_bg(2),q_bg(3),q_bg(4),V_Kg(1),V_Kg(2),V_Kg(3));
y = uint8(ystr);

end