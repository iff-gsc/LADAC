function traj  = trajFromCubicSpline(traj, waypoints, num_of_waypoints, varargin)

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Fabian Guecker
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

coder.extrinsic('warning')
warning('The function trajFromCubicSpline is deprecated and will be removed in the future, Please use trajFromWaypoints');

cycle = true;

if ~isempty(varargin)
    cycle= logical(varargin{1});
end

array_bounds = [num_of_waypoints, traj.num_sections_max, size(waypoints,2)];
num_of_splines = min(array_bounds);

if(cycle)
    points = zeros(3, num_of_splines+3);
    % Copy all points
    points(:, 2:end-2) = waypoints(:,1:num_of_splines);
    
    % Attach first point to the end
    points(:, end-1) = waypoints(:,1);
    
    % Attach first and last points slope
    points(:, 1) = (waypoints(:,2) - waypoints(:,end)) / (4/3);
    points(:, end) = points(:, 1);

else
    points = waypoints(:,1:num_of_splines);
    num_of_splines = num_of_splines-1;
end

% Configure trajectory struct with information about the path 
traj.num_sections_set   = num_of_splines;
traj.is_repeated_course = cycle;
traj.polynomial_degree  = 3;

pp = spline(0:num_of_splines, points);

if coder.target('MATLAB')
    % Executing in MATLAB, use unmkpp to return struct with coefs and
    % breaks, to get the same result as with code generation
    [~,coefs,~,~,~] = unmkpp(pp);
    pp.coefs = reshape(coefs,3,num_of_splines,4);
end

for i=1:num_of_splines
    for j=1:4
        traj.sections(i).pos_x(j) = pp.coefs(1, i, j);
        traj.sections(i).pos_y(j) = pp.coefs(2, i, j);
        traj.sections(i).pos_z(j) = pp.coefs(3, i, j);
    end
end

traj = trajSetArcLength(traj);


end
