function [section_id, t] = trajSweepAlong(traj, section_id_start, t_start, dist)
% trajSweepAlong returns the point at the given distance away from
% start point along the path.
%
% Inputs:
%   traj                trajectory struct, see trajInit
%
%   section_id_start    Section from which the sweep is started
%                       (scalar), integer
%
%   t_start             dimensionless time parameter, from which the sweep
%                       is started
%                       (scalar), range [0-1]
%
%   dist                arc length to sweep along the trajectory
%                       (scalar)
%
% Outputs:
%
%   section_id          Section of the trajectory that is the specified arc
%                       length away from the starting point.
%                       (scalar), integer
%
%   t                   dimensionless time parameter that is the specified
%                        arc length away from the starting point.
%                       (scalar), integer
%
% Syntax:
%   [section_id, t] = trajSweepAlong(traj, section_id_start, t_start, dist)
%
% See also: trajInit, trajGetMatch, trajCreateFromWaypoints
%

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Fabian Guecker
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% Initialization
t = -1;
section_id = -1;
active_section = section_id_start;

% Check if the distance is a multiple of the total length of the trajectory
dist_achieved =  fix(divideFinite(dist,traj.arc_length)) * traj.arc_length;
dist_remaining = abs(dist - dist_achieved);

% Check the direction of travel
if(sign(dist) >= 0)
    
    %run forward once through the trajectory
    for k = 0:traj.num_sections_set
        
        % Check if the end has been reached, if yes start at the beginning
        index = active_section + k;
        if(index > traj.num_sections_set)
            index = index - traj.num_sections_set;
        end
        
        % Get the arc length of the current section
        arc_length = traj.sections(index).arc_length;
        
        % Get the still available arc length of the section
        [arc_len_start, arc_len_start_diff] = ...
            trajSectionGetArcLength(traj.sections(index), t_start);
        
        dist_available = arc_length - arc_len_start;
        
        % Check if the current section is long enough
        if(dist_remaining <= dist_available)
            %% Newton-Iteration
            % The arc length applies only to the entire piece. The distance
            % covered and the run parameter are non-linearly dependent on
            % each other. Therefore, the exact value is determined with a
            % few Newton iterations.
            
            section_id(:) = index;
            act_sec = traj.sections(section_id);
            
            t(:) = t_start;
            arc_len = arc_len_start;
            arc_len_diff = arc_len_start_diff;
            
            % Inital Newton step
            t(:) = t - (arc_len - dist_remaining - arc_len_start) / arc_len_diff;
            
            for i = 1:2
                % Update arc length and derivative
                [arc_len, arc_len_diff] = ...
                    trajSectionGetArcLength(act_sec, t);
                
                % Newton iteration step
                t(:) = t - (arc_len - dist_remaining - arc_len_start) / arc_len_diff;
            end
            
            break;
        else
            % Subtract the available distance entirely
            dist_remaining = dist_remaining - dist_available;
            t_start(:) = 0;
        end
        
    end
    
else
    
    % run backwards through the trajectory once
    for k = 0:traj.num_sections_set
        
        % Check if the end has been reached, if yes start at the beginning
        index = active_section - k;
        if(index < 1)
            index = index + traj.num_sections_set;
        end
        
        % Get the still available arc length of the section
        [arc_len_start, arc_len_start_diff] = ...
            trajSectionGetArcLength(traj.sections(index), t_start);
        
        dist_available = arc_len_start;
        
        % Check if the current section is long enough
        if(dist_remaining <= dist_available)
            %% Newton-Iteration
            % The arc length applies only to the entire piece. The distance
            % covered and the run parameter are non-linearly dependent on
            % each other. Therefore, the exact value is determined with a
            % few Newton iterations.
            
            section_id(:) = index;
            act_sec = traj.sections(section_id);
            
            t(:) = t_start;
            arc_len = arc_len_start;
            arc_len_diff = arc_len_start_diff;
            
            % Inital Newton step
            t(:) = t - (dist_remaining + arc_len - arc_len_start) / arc_len_diff;
            
            for i = 1:2
                % Update arc length and derivative
                [arc_len, arc_len_diff] = ...
                    trajSectionGetArcLength(act_sec, t);
                
                % Newton iteration step
                t(:) = t - (dist_remaining + arc_len - arc_len_start) / arc_len_diff;
            end                                
            
            break;
            
        else
            % Subtract the available distance entirely
            dist_remaining = dist_remaining - dist_available;
            t_start(:) = 1;
        end
    end
end
end