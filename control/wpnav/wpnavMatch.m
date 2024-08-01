function [p_match,wp_idx,stage,t,d] = wpnavMatch( waypoints, wp_radius, wp_idx, stage, p )
% wpnavMatch match the current position to the specified waypoint track
%   This function performs the matching of a given current point p to the
%   specified waypoint track and returns the matched position.
%   The inputs wp_idx and stage define the current active waypoint and if
%   we are currently on a line segment or on a circle segment (that
%   connects two lines). wp_idx and stage may be updated by this function
%   if the end of the current segment way reached since the last call.
% 
% Syntax:
%   [p_match,wp_idx,stage,t,d] = wpnavMatch( waypoints, wp_radius, wp_idx, stage, p )
% 
% Inputs:
%   waypoints           Waypoints (3xN array, where N is the number of
%                       wapyoints) in g frame (north-east-down), in m
%   wp_radius           Waypoint radius (scalar), in m
%   wp_idx              Waypoint index (scalar): Next waypoint on the
%                       waypoint track
%   stage               Stage (scalar, 0 or 1): 0 if we are on a circle
%                       segment, 1 if we are on a line segment
%   p                   Current position (3x1 array) in g frame
%                       (north-east-down), in m
% 
% Outputs:
%   p_match             Matched position (3x1 array) on line segment or
%                       circle segment on the waypoint track in g frame
%                       (north-east-down), in m
%   wp_idx              Waypoint index (scalar): Next waypoint on the
%                       waypoint track
%   stage               Stage (scalar, 0 or 1): 0 if we are on a circle
%                       segment, 1 if we are on a line segment
%   t                   Non-dimensional time (scalar): 0 at the start of
%                       the segment, 1 at the end of the segment
%   d                   Position error (scalar): distance from the current
%                       position to the matched position, in m
% 
% See also:
%   wpnavMatchLine, wpnavMatchCircSeg

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2024 Yannic Beyer
%   Copyright (C) 2024 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

num_wp = size(waypoints,2);

is_cycle = true;

p_match = zeros(3,1,superiorfloat(waypoints));
t = zeros(1,1,superiorfloat(waypoints));
d = zeros(1,1,superiorfloat(waypoints));

% check mission stage
% if wp_idx < 2
%     wp_idx = 2;
% end
% if wp_idx == 2 && stage == 0
%     stage = 1;
% end
if wp_idx > num_wp
    wp_idx(:) = num_wp;
end
if wp_idx < 1
    wp_idx(:) = 1;
end

while true
    
    if wp_idx == 2
        is_initial_line = true;
    else
        is_initial_line = false;
    end
    if wp_idx == num_wp
        is_last_line = true;
    else
        is_last_line = false;
    end
    
    % circle segment
    if stage == 0
        idx3 = fixIdx([wp_idx-2,wp_idx-1,wp_idx],num_wp);
        circ_seg = wpnavCircSeg(waypoints(:,idx3),wp_radius);
        [p_match(:),t(:),d(:)] = wpnavMatchCircSeg(circ_seg,p);

        
        %D = divideFinite( dot( p-waypoints(:,wp_idx-1), circ_seg.end-waypoints(:,wp_idx-1) ), norm(circ_seg.end-waypoints(:,wp_idx-1),2) );
        if t > 1% || D > norm(circ_seg.end-waypoints(:,wp_idx-1),2)
            stage(:) = 1;
        else
            break;
        end
    end
    
    % straigth line
    if stage == 1
        if is_initial_line
            p1 = waypoints(:,1);
        else
            idx3 = fixIdx([wp_idx-2,wp_idx-1,wp_idx],num_wp);
            circ_seg_1 = wpnavCircSeg(waypoints(:,idx3),wp_radius);
            p1 = circ_seg_1.end;
        end
        if is_last_line
            if is_cycle
                idx2 = fixIdx([wp_idx-1,wp_idx],num_wp);
                circ_seg_2 = wpnavCircSeg([waypoints(:,idx2),waypoints(:,1)],wp_radius);
                p2 = circ_seg_2.start;
            else
                p2 = waypoints(:,end);
            end
        else
            idx3 = fixIdx([wp_idx-1,wp_idx,wp_idx+1],num_wp);
            circ_seg_2 = wpnavCircSeg(waypoints(:,idx3),wp_radius);
            p2 = circ_seg_2.start;
        end
        line_length = norm( p1 - p2, 2);
        if line_length < 1
            stage(:) = 0;
            wp_idx(:) = wp_idx + 1;
        else
            [p_match(:),t(:),d(:)] = wpnavMatchLine(p1,p2,p);
            if t > 1 && norm(p-p2,2) < wp_radius
                stage(:) = 0;
                wp_idx(:) = wp_idx + 1;
            else
                break;
            end
        end
    end
    if wp_idx > num_wp
        if is_cycle
            wp_idx(:) = 1;
        else
            % To do: What to do if last waypoint is reached but no cycle is
            % desired?
            break;
        end
    end
end

end

function idx = fixIdx(idx,N)
for i = 1:length(idx)
    if idx(i) < 1
        idx(i) = N + idx(i);
    end
end
end
