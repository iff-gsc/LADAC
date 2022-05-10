function traj = trajFromWaypoints(traj, waypoints, degree, cycle)
% trajFromWaypoints computes the trajectory from given waypoints
%   The function returns the filled trajectory struct representing a flight
%   path given by the waypoints
%
% Inputs:
%   traj            trajectory struct, see trajInit
%
%   points          trajectory waypoints [x; y; z] in local geodetic
%                   coordinate system
%                   (3xN vector), in m
%
%   degree 		    degree of internal polynomial representation this value
%                   should be an odd number to ensure symmetrically
%                   boundary conditions on every knot point (1,3,5,...)
%                   (scalar), dimensionless
%
%   cycle           enable automatic repetition of the given course
%
% Outputs:
%
%   traj            trajectory struct, see trajInit
%
% Syntax:
%   [traj] = trajFromWaypoints(traj, points, degree, cycle)
%
% See also: trajInit, traj_from_waypoints_example
%

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Fabian Guecker
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% Check if the polynomial degree is even, if yes increase it by one to get 
% the same number of constraints on the left and right side.
if(~mod(degree,2))
    degree = degree + 1;
end
%num_of_splines = single(0);
% Calculate the coefficients
[coeffs_x, num_of_splines] = polyInterpolation(waypoints(1,:), ...
                             degree, cycle, 0, 0);
[coeffs_y, ~] = polyInterpolation(waypoints(2,:), degree, cycle, 0, 0);
[coeffs_z, ~] = polyInterpolation(waypoints(3,:), degree, cycle, 0, 0);

% Configure trajectory struct with information about the path 
traj.num_sections_set(:)   = num_of_splines;
traj.is_repeated_course = cycle;
traj.polynomial_degree  = degree;

% Copy the coefficients into the spline trajectory
num_of_coeffs = degree + 1;

% Copty the coefficients into the traj struct
for i=1:num_of_splines
    idx_beg = num_of_coeffs*(i-1)+1;
    idx_end = num_of_coeffs*(i);
    traj.sections(i).pos_x(:) = coeffs_x(idx_beg:idx_beg+5);
    traj.sections(i).pos_y(:) = coeffs_y(idx_beg:idx_beg+5);
    traj.sections(i).pos_z(:) = coeffs_z(idx_beg:idx_beg+5);
end

% Update arc length for each section
traj = trajSetArcLength(traj);

end


