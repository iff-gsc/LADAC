function [section_idx_ret, error, t_ret] = trajGetMatchEnhanced(traj, position, active_section)
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
%   SPDX-License-Identifier: GPL-3.0-only
%
%   Copyright (C) 2020-2022 Fabian Guecker
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

first_checked_section = 1;
last_checked_section = traj.num_sections_set;

traj_section = trajGetSection(traj, 1);

t = zeros(1,superiorfloat(position));
section_idx = ones(1,superiorfloat(position));
error = norm(position - trajSectionGetPos(traj_section, 0));

degree = length(traj.sections(1).pos_x);

t_array = zeros(1,superiorfloat(position));

trajSection = trajGetSection(traj, section_idx);

xf = position(1);
yf = position(2);
zf = position(3);

section_idx_ret = 0;
t_ret = 0;

for k = 1:last_checked_section
    
    t = 0;
    %trajSection = traj.sections(k);
        
    section_idx = k; 
    
    %% Projection Iteration
    
    for i=1:20
        
        trajSection = trajGetSection(traj, section_idx);    
        
        % Universal calculation method        
        
        f = trajSectionGetPos(trajSection, t);
        [f_dt, ~] = trajSectionGetDerivatives(trajSection, t);
        
        %eval_fun = 2*(f_dt(1)*(f(1)-xf) + f_dt(2)*(f(2)-yf) + f_dt(3)*(f(3)-zf));
        
        distance = norm(f-position);
        %disp(distance);
        
        %eval_fun_dt = 2*(f_dt2(1)*(f(1)-xf) + f_dt2(2)*(f(2)-yf) + f_dt2(3)*(f(3)-zf)...
        %    + f_dt(1)^2 + f_dt(2)^2 + f_dt(3)^2);
        
        eval_fun = (f_dt(1)*(f(1)-xf) + f_dt(2)*(f(2)-yf) + f_dt(3)*(f(3)-zf));
        
        % Check for convergence
        
        %if abs(eval_fun_dt) < 1e-16
        %    eval_fun_dt(:) = 1e-8;
        %end
        
        % One Newton step
        %t = t - 2*divideFinite(eval_fun, eval_fun_dt);
        
        step = -eval_fun / norm(f_dt)^2;
        
        step = min(max(step, -0.25), 0.25);
        
        t = t + step;
        
        
        % Break loop
        if(abs(eval_fun) < 1e-12)
            
            if( distance < error)
                error = distance;
                section_idx_ret = section_idx;
                t_ret = min(max(t,0),1);
            end
            break;
        end
        
        % Step to previous/next section
        if(t < 0) || (t > 1.00)
            if(t < 0)
                t = max(t+1, 1.00);
                section_idx = section_idx - 1;
                if(section_idx < 1)
                    if traj.is_repeated_course
                        section_idx = traj.num_sections_set;
                    else
                        section_idx = ones(1,superiorfloat(position));
                        t = zeros(1,superiorfloat(position));
                    end
                end
            else
                t = min(t-1, 0.00);
                section_idx = section_idx + 1;
                if(section_idx > traj.num_sections_set)
                    if traj.is_repeated_course
                        section_idx = ones(1,superiorfloat(position));
                    else
                        section_idx = traj.num_sections_set;
                        t = ones(1,superiorfloat(position));
                    end
                end
            end
            
            trajSection = trajGetSection(traj, section_idx);
            
        end
        
    end
    
    
   % disp(i)
    %% End 
    
    
    
end





end