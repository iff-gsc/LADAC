function traj_section = trajGetSection( traj, varargin )
% trajGetSection returns a trajectory section struct.
% 
% Inputs:
%   traj            trajectory struct, see trajInit
%   section_idx     the index of the section to be returned
%
% Outputs:
%   traj_section    trajectory section struct, see trajSectionInit
%
% Syntax: 
%   traj = trajGetSection( traj )
%   traj = trajGetSection( traj, section_idx )
%
% See also: trajInit, trajSectionSet
%

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Fabian Guecker
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

section_idx = 0 ;

if isempty(varargin)
    section_idx(:) = traj.active_section;
else
    section_idx(:) = varargin{1};
end

if section_idx > traj.num_sections_set || section_idx < 1
    %warning('Invalid section index. First section will be returned.');
    section_idx(:) = 1;    
end

traj_section = traj.sections(section_idx);

end