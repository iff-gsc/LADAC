function traj = trajSetMatch(traj, position, active_section)
% trajSetMatch sets the nearest point on the trajectory.
%   The current trajectory section and the dimensionless time
%   parameter are set to the point with minimum euclidean distance in
%   respect to the given vehicle position.
%
% Inputs:
%   traj                trajectory struct, see trajInit
%
%   position            vehicle position [x; y; z] in local geodetic
%                       coordinate system
%                       (3x1 vector), in m
%
%   active_section      *placeholder for later use*
%                       current section of the trajectory
%                       (scalar), dimensionless
%
% Outputs:
%   traj                trajectory struct, see trajInit
%
% Syntax:
%   traj = trajSetMatch(traj, position, active_section)
%
% See also: trajGetMatch, trajInit
%

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Fabian Guecker
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

[ active_section, t ] = trajGetMatch( traj, position, active_section );

traj.active_section = active_section;
traj.current_time = t;

end