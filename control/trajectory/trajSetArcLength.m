function [traj] = trajSetArcLength(traj)
% trajSetArcLength update the arc length of a trajectory
%   The function returns a trajectory struct with updated total arc length
%   and linear distance between the waypoints of the trajectory.
%
% Inputs:
%   traj        	trajectory struct
%
% Outputs:
%   traj            trajectory struct, with updated arc length and
%                   airline distance between the waypoints
%
% Syntax:
%   [acc] = trajGetAcc(traj)
%
% See also: trajGetPos, trajGetVel, trajSectionGetArcLength,
%   trajCreateFromWaypoints
%

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Fabian Guecker
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% trajectory arc length
total_arc_length = 0;

% trajectory airline distance 
total_distance   = 0;

% Step through all trajectory sections
for k=1:traj.num_sections_set

    % Get trajectory section struct
    traj_section = trajGetSection(traj,k);
    
    % Get the first and last point in the current section
    pos_last = trajSectionGetPos(traj_section,1);
    pos_first = trajSectionGetPos(traj_section,0);
    
    % Calculate airline distance of current section
    distance = norm(pos_last-pos_first,2);
    
    % Numerical integration of the arc length of the current section
    arc_length = trajSectionGetArcLength(traj_section,1);
    
    % Update the arc length and airline distance for the current section
    traj.sections(k).arc_length(:) = arc_length;
    traj.sections(k).distance(:) = distance;
    
    % Adding the arc length and the airline distance to the total values 
    % of the trajectory
    total_arc_length(:) = total_arc_length + arc_length;
    total_distance(:)   = total_distance + distance;
end

% Write values into the trajectory struct
traj.distance(:)   = total_distance;
traj.arc_length(:) = total_arc_length;

end