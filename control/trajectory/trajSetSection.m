function traj = trajSetSection( traj, traj_section, varargin )
% trajSectionSet copy a trajectory section into a trajectory.
%   The function writes the trajectory section struct traj_section into
%   the trajectory struct traj. Various tests are performed, to ensure that
%   no invalid sections are defined.
% 
% Inputs:
%   traj            trajectory struct, see trajInit
%   traj_section    trajectory section struct, see trajSectionInit
%   section_idx 	index of the section to be set
% 
% Outputs:
%   traj            trajectory struct, see trajInit
%
% Syntax: 
%   traj = trajSetSection( traj, traj_section )
%   traj = trajSetSection( traj, traj_section, section_idx )
%
% See also: trajInit, trajSectionSet
%

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Fabian Guecker
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

section_idx = 0;

if isempty(varargin)
    % increment
    if traj.num_sections_set < traj.num_sections_max
        traj.num_sections_set = traj.num_sections_set + 1;
    else
        warning('Buffer size reached. Trajectory section was not set.');
    end
    section_idx(:) = traj.num_sections_set;
else
    section_idx(:) = varargin{1};
    if section_idx > traj.num_sections_set || section_idx < 1
        warning('Invalid section index. Trajectory section was not set.');
        return;
    end
end

traj.sections(section_idx) = traj_section;

end