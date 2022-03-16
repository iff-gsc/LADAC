function traj = trajInit( num_sections_max, varargin )
% trajInit defines a trajectory struct.
%   The trajectory struct contains a buffer of several trajectory sections
%   which define the trajectory with splines.
%   
% Inputs:
%   num_sections_max    maximum number of splines of the trajectory
%
%   degree              Degree of a polynomials in the trajectory section
%                       (optional, default = 3)
%
% Outputs:
%   traj        trajectory coefficent struct
%               with              
%               traj.num_sections_max   maximum number of sections (buffer)
%               traj.num_sections_set   number of set sections inside
%                                       buffer
%               traj.sections           (N-1 x 1) array of trajectory
%                                       section structs, see
%                                       trajSectionInit
%               traj.active_section     the active section (from matching)
%               traj.current_time       the current dimensionless time on
%                                       on the section (from matching)
%
% Syntax: 
%   traj = trajInit(waypoints) 
%
% See also: trajGetPos, trajGetVel, trajGetAcc
%

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Fabian Guecker
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% Set default degree of the polynominal
degree = 3;

if (~isempty(varargin))
    degree = max(real(varargin{1}), 2);
end   

traj = struct( ...
    'num_sections_max', num_sections_max, ...
    'num_sections_set', 0, ...
    'sections', repmat( trajSectionInit(degree), num_sections_max, 1 ), ...
    'active_section', 0, ...
    'current_time', 0, ...
    'arc_length', 0, ...
    'distance', 0,...
    'is_repeated_course', false, ...
    'polynomial_degree', degree ...
    );

end