function is_valid = trajValidateWaypoints(waypoints, num_wp, cycle)
% trajValidateWaypoints checks that the waypoints form a valid trajectory
%   The function returns true if the given waypoints form a valid
%   trajectory, otherwise it returns false
%
% Inputs:
%   points          trajectory waypoints [x; y; z] in local geodetic
%                   coordinate system
%                   (3xN vector), in m
%
%   num_wp          the total number of waypoints
%                   (scalar), dimensionless
%
%   cycle           enable automatic repetition of the given course
%
% Outputs:
%
%   valid           hold result if the waypoints form a valid trajectory
%                   (logical), dimensionless
%
% Syntax:
%   is_valid = trajValidateWaypoints(traj, waypoints, degree, cycle)
%
% See also: trajInit
%

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Fabian Guecker
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% Set default value to false, perform checks to sort out errors.
% If the end of function is reached, no errors occured.
is_valid = false;

% At least 3 Waypoints are needed for a course
if num_wp < 3
    return;
end
 
is_valid = true;

end