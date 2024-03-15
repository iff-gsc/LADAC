function [pos,vel,acc] = wpnavCmd(waypoints,wp_radius,wp_idx,stage,t,p)

[wp_idx,stage,t,d] = wpnavMatch(waypoints,wp_radius,wp_idx,stage,t,p);

end