function [section_idx, error, t] = trajGetMatchEnhanced(traj, position, active_section)
% trajGetMatch computes the nearest point on the trajectory
%   The function returns the point on the trajectory that has minimum
%   euclidean norm in respect to the given position.
%
% Inputs:
%   traj            trajectory struct, see trajInit
%
%   position        vehicle position [x; y; z] in local geodetic
%                   coordinate system
%                   (3x1 vector), in m
%
%   active_section 	*placeholder for later use*
%                   current section of the trajectory
%                  	(scalar), dimensionless
% Outputs:
%
%   section_idx     index of the trajectory section that contains the point
%                   of minimum eulidean distance
%
%   error           the minimum euclidean distance between the given
%                   position and the trajectory
%                   (scalar), in m
%
%   t               dimensionless time parameter that correnspons to the
%                   trajectory point of minimum eulidean distance
%                	(scalar), [0-1]
%
% Syntax:
%   [section_idx, error, t] = trajGetMatch(traj, position, active_section)
%
% See also: trajGetMatch, trajCreateFromWaypoints
%

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Fabian Guecker
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

first_checked_section = 1;
last_checked_section = traj.num_sections_set;

traj_section = trajGetSection(traj, 1);

t = 0.0;
section_idx = 1;
error = norm(position - trajSectionGetPos(traj_section, 0));

degree = length(traj.sections(1).pos_x);

resolution = 2*degree;
t_array = linspace(0, 1, resolution);

for k = first_checked_section:last_checked_section
    
    traj_section = traj.sections(k);
    
    % Calculate x-positon with t
    px = polyval(traj_section.pos_x, t_array);
    
    % Calculate y-positon with t
    py = polyval(traj_section.pos_y, t_array);
    
    % Calculate z-positon with t
    pz = polyval(traj_section.pos_z, t_array);
    
    % return position vector
    curr_pos = [px; py; pz];
    
    for j = 1:resolution
        curr_pos(:,j) = curr_pos(:,j) - position;
    end
      
    error_array = vecnorm(curr_pos);
    
    %error_array = vecnorm(curr_pos-position);
    
    [curr_error, idx] = min(error_array);
    
    if( curr_error < error)
        error = curr_error;
        section_idx = k;
        t = t_array(idx);
    end
    
end

trajSection = trajGetSection(traj, section_idx);

xf = position(1);
yf = position(2);
zf = position(3);

plot_enable = 0;

for i=1:100
    
    if(plot_enable)
        hold on
        plot3(xf, yf, -zf, 'rx')
        hold off
        
        pos_m = trajSectionGetPos(trajSection, t);
        hold on
        plot3(pos_m(1), pos_m(2), -pos_m(3), 'bx')
        hold off
        
    end
    
    % Universal calculation method
    
    f = trajSectionGetPos(trajSection, t);
    [f_dt, f_dt2] = trajSectionGetDerivatives(trajSection, t);
    
    eval_fun = 2*(f_dt(1)*(f(1)-xf) + f_dt(2)*(f(2)-yf) + f_dt(3)*(f(3)-zf));
    
    eval_fun_dt = 2*(f_dt2(1)*(f(1)-xf) + f_dt2(2)*(f(2)-yf) + f_dt2(3)*(f(3)-zf)...
        + f_dt(1)^2 + f_dt(2)^2 + f_dt(3)^2);
    
    % Check for convergence
    
    if abs(eval_fun_dt) < 1e-16
        eval_fun_dt = 1e-8;
    end
    
    % One Newton step
    t = t - eval_fun / eval_fun_dt;
    
    % Break loop
    if(abs(eval_fun) < 1e-12)
        break;
    end
    
    % Step to previous/next section
    if(t < 0) || (t > 1)
        if(t < 0)
            t = max(t+1, 0.95);
            section_idx = section_idx - 1;
            if(section_idx < 1)
                if traj.is_repeated_course
                    section_idx = traj.num_sections_set;
                else
                    section_idx = 1;
                    t = 0;
                end
            end
        else
            t = min(t-1, 0.05);
            section_idx = section_idx + 1;
            if(section_idx > traj.num_sections_set)
                if traj.is_repeated_course
                    section_idx = 1;
                else
                    section_idx = traj.num_sections_set;
                    t = 1;
                end
            end
        end
        
        trajSection = trajGetSection(traj, section_idx);
        
    end
    
end

%disp(i);

t = min(max(t,0),1);

end