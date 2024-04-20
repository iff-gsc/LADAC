function [p_match,wp_idx,stage,t,d] = wpnavMatch(waypoints,wp_radius,wp_idx,stage,p)

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
    wp_idx = num_wp;
end
if wp_idx < 1
    wp_idx = 1;
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
        idx3 = fixIdx(wp_idx-2:wp_idx,num_wp);
        circ_seg = wpnavCircSeg(waypoints(:,idx3),wp_radius);
        [p_match(:),t(:),d(:)] = wpnavMatchCircSeg(circ_seg,p);

        
        %D = divideFinite( dot( p-waypoints(:,wp_idx-1), circ_seg.end-waypoints(:,wp_idx-1) ), norm(circ_seg.end-waypoints(:,wp_idx-1),2) );
        if t > 1% || D > norm(circ_seg.end-waypoints(:,wp_idx-1),2)
            stage = 1;
        else
            break;
        end
    end
    
    % straigth line
    if stage == 1
        if is_initial_line
            p1 = waypoints(:,1);
        else
            idx3 = fixIdx(wp_idx-2:wp_idx,num_wp);
            circ_seg_1 = wpnavCircSeg(waypoints(:,idx3),wp_radius);
            p1 = circ_seg_1.end;
        end
        if is_last_line
            if is_cycle
                idx2 = fixIdx(wp_idx-1:wp_idx,num_wp);
                circ_seg_2 = wpnavCircSeg([waypoints(:,idx2),waypoints(:,1)],wp_radius);
                p2 = circ_seg_2.start;
            else
                p2 = waypoints(:,end);
            end
        else
            idx3 = fixIdx(wp_idx-1:wp_idx+1,num_wp);
            circ_seg_2 = wpnavCircSeg(waypoints(:,idx3),wp_radius);
            p2 = circ_seg_2.start;
        end
        line_length = norm( p1 - p2, 2);
        if line_length < 1
            stage = 0;
            wp_idx = wp_idx + 1;
        else
            [p_match(:),t(:),d(:)] = wpnavMatchLine(p1,p2,p);
            if t > 1 && norm(p-p2,2) < wp_radius
                stage = 0;
                wp_idx = wp_idx + 1;
            else
                break;
            end
        end
    end
    if wp_idx > num_wp
        if is_cycle
            wp_idx = 1;
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
