function [acc_cmd,section_id,traj_appr] = trajGetApproachCurve...
    (traj, section_id_start, t_start, look_forward_distance, inital_pos, inital_vel,tension, weight)

% Current position and velocity vector
inital_vel = inital_vel / norm(inital_vel);

% Run the given distance along the path starting from the nearest point
[section_id, t] = trajSweepAlong(traj, section_id_start, t_start, look_forward_distance);
active_section = trajGetSection(traj,section_id);

% Get target position and velocity vector
[target_pos] = trajSectionGetPos(active_section, t);
[target_vel] = trajSectionGetDerivatives(active_section, t);

% Change the velocity vectors to unit vectors
inital_vel   =  inital_vel / norm(inital_vel) * weight(1);
target_vel   =  target_vel / norm(target_vel) * weight(2);

% Fill the structure
points = {{inital_pos, inital_vel}, {target_pos', target_vel'}};

% Use the same tension as the trajectory to get a subset of it if
% there is a perfect match.
%tension = 1.0;

% Calculate approach trajectory
[traj_appr] = hobbysplines_Simulink(points, tension);

%%

traj_appr_sec = trajGetSection(traj_appr, 1);
%traj_appr_pos = trajectorySectionGetPos(traj_appr_sec, 0);
vel = norm(inital_vel);
acc_cmd = trajSectionGetAcc(traj_appr_sec, vel, 0);
%T, B, N] = trajectorySectionGetFrenetSerret(traj_appr_section, 0);

end