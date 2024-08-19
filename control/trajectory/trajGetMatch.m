function [section_idx, error, t] = trajGetMatch(traj, position, active_section)
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

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Fabian Guecker
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

xf = position(1);
yf = position(2);
zf = position(3);

% first_checked_section = max(1,active_section-1);
% last_checked_section = min(traj.num_sections_set,active_section+3);
first_checked_section = 1;
last_checked_section = traj.num_sections_set;

section_errors = zeros(last_checked_section-first_checked_section+1, 1,superiorfloat(position));
section_t      = zeros(last_checked_section-first_checked_section+1, 1,superiorfloat(position));

traj_degree = traj.polynomial_degree;

for k = first_checked_section:last_checked_section
    
    % Default for degree of 3
    no_of_points = 1;
    
    P = zeros(1,18,superiorfloat(position));
    
    traj_section = trajGetSection(traj,k);
    
    % Polynomial coefficents
    x = traj_section.pos_x;
    y = traj_section.pos_y;
    z = traj_section.pos_z;
    
    if(traj_degree == 3)
        
        no_of_points = 7;
        
        P(13) = 3*x(1)^2 + 3*y(1)^2 + 3*z(1)^2;
        P(14) = 5*x(1)*x(2) + 5*y(1)*y(2) + 5*z(1)*z(2);
        P(15) = 2*x(2)^2 + 2*y(2)^2 + 2*z(2)^2 + 4*x(1)*x(3) + 4*y(1)*y(3) + 4*z(1)*z(3);
        P(16) = 3*x(1)*x(4) + 3*x(2)*x(3) - 3*x(1)*xf + 3*y(1)*y(4) + 3*y(2)*y(3) - 3*y(1)*yf + 3*z(1)*z(4) + 3*z(2)*z(3) - 3*z(1)*zf;
        P(17) = x(3)^2 + y(3)^2 + z(3)^2 + 2*x(2)*x(4) - 2*x(2)*xf + 2*y(2)*y(4) - 2*y(2)*yf + 2*z(2)*z(4) - 2*z(2)*zf;
        P(18) = x(3)*x(4) - x(3)*xf + y(3)*y(4) - y(3)*yf + z(3)*z(4) - z(3)*zf;
        
        if(abs(P(13)) <= eps)
            P(13) = eps;
        end
        
    end
    
    
    if(traj_degree == 5)
        
        no_of_points = 11;
        
        P(9) = 5*x(1)^2 + 5*y(1)^2 + 5*z(1)^2;
        P(10) = 9*x(1)*x(2) + 9*y(1)*y(2) + 9*z(1)*z(2);
        P(11) = 4*x(2)^2 + 4*y(2)^2 + 4*z(2)^2 + 8*x(1)*x(3) + 8*y(1)*y(3) + 8*z(1)*z(3);
        P(12) = 7*x(1)*x(4) + 7*x(2)*x(3) + 7*y(1)*y(4) + 7*y(2)*y(3) + 7*z(1)*z(4) + 7*z(2)*z(3);
        P(13) = 3*x(3)^2 + 3*y(3)^2 + 3*z(3)^2 + 6*x(1)*x(5) + 6*x(2)*x(4) + 6*y(1)*y(5) + 6*y(2)*y(4) + 6*z(1)*z(5) + 6*z(2)*z(4);
        P(14) = 5*x(1)*x(6) + 5*x(2)*x(5) + 5*x(3)*x(4) - 5*x(1)*xf + 5*y(1)*y(6) + 5*y(2)*y(5) + 5*y(3)*y(4) - 5*y(1)*yf + 5*z(1)*z(6) + 5*z(2)*z(5) + 5*z(3)*z(4) - 5*z(1)*zf;
        P(15) = 2*x(4)^2 + 2*y(4)^2 + 2*z(4)^2 + 4*x(2)*x(6) + 4*x(3)*x(5) - 4*x(2)*xf + 4*y(2)*y(6) + 4*y(3)*y(5) - 4*y(2)*yf + 4*z(2)*z(6) + 4*z(3)*z(5) - 4*z(2)*zf;
        P(16) = 3*x(3)*x(6) + 3*x(4)*x(5) - 3*x(3)*xf + 3*y(3)*y(6) + 3*y(4)*y(5) - 3*y(3)*yf + 3*z(3)*z(6) + 3*z(4)*z(5) - 3*z(3)*zf;
        P(17) = x(5)^2 + y(5)^2 + z(5)^2 + 2*x(4)*x(6) - 2*x(4)*xf + 2*y(4)*y(6) - 2*y(4)*yf + 2*z(4)*z(6) - 2*z(4)*zf;
        P(18) = x(5)*x(6) - x(5)*xf + y(5)*y(6) - y(5)*yf + z(5)*z(6) - z(5)*zf;
        
        if(abs(P(9)) <= eps)
            P(9) = eps;
        end
        
    end
    
    
%     if(traj_degree == 7)
%         
%         no_of_points = 15;
%         
%         P(5) = 7*x(1)^2 + 7*y(1)^2 + 7*z(1)^2;
%         P(6) = 13*x(1)*x(2) + 13*y(1)*y(2) + 13*z(1)*z(2);
%         P(7) = 6*x(2)^2 + 6*y(2)^2 + 6*z(2)^2 + 12*x(1)*x(3) + 12*y(1)*y(3) + 12*z(1)*z(3);
%         P(8) = 11*x(1)*x(4) + 11*x(2)*x(3) + 11*y(1)*y(4) + 11*y(2)*y(3) + 11*z(1)*z(4) + 11*z(2)*z(3);
%         P(9) = 5*x(3)^2 + 5*y(3)^2 + 5*z(3)^2 + 10*x(1)*x(5) + 10*x(2)*x(4) + 10*y(1)*y(5) + 10*y(2)*y(4) + 10*z(1)*z(5) + 10*z(2)*z(4);
%         P(10) = 9*x(1)*x(6) + 9*x(2)*x(5) + 9*x(3)*x(4) + 9*y(1)*y(6) + 9*y(2)*y(5) + 9*y(3)*y(4) + 9*z(1)*z(6) + 9*z(2)*z(5) + 9*z(3)*z(4);
%         P(11) = 4*x(4)^2 + 4*y(4)^2 + 4*z(4)^2 + 8*x(1)*x(7) + 8*x(2)*x(6) + 8*x(3)*x(5) + 8*y(1)*y(7) + 8*y(2)*y(6) + 8*y(3)*y(5) + 8*z(1)*z(7) + 8*z(2)*z(6) + 8*z(3)*z(5);
%         P(12) = 7*x(1)*x(8) + 7*x(2)*x(7) + 7*x(3)*x(6) + 7*x(4)*x(5) - 7*x(1)*xf + 7*y(1)*y(8) + 7*y(2)*y(7) + 7*y(3)*y(6) + 7*y(4)*y(5) - 7*y(1)*yf + 7*z(1)*z(8) + 7*z(2)*z(7) + 7*z(3)*z(6) + 7*z(4)*z(5) - 7*z(1)*zf;
%         P(13) = 3*x(5)^2 + 3*y(5)^2 + 3*z(5)^2 + 6*x(2)*x(8) + 6*x(3)*x(7) + 6*x(4)*x(6) - 6*x(2)*xf + 6*y(2)*y(8) + 6*y(3)*y(7) + 6*y(4)*y(6) - 6*y(2)*yf + 6*z(2)*z(8) + 6*z(3)*z(7) + 6*z(4)*z(6) - 6*z(2)*zf;
%         P(14) = 5*x(3)*x(8) + 5*x(4)*x(7) + 5*x(5)*x(6) - 5*x(3)*xf + 5*y(3)*y(8) + 5*y(4)*y(7) + 5*y(5)*y(6) - 5*y(3)*yf + 5*z(3)*z(8) + 5*z(4)*z(7) + 5*z(5)*z(6) - 5*z(3)*zf;
%         P(15) = 2*x(6)^2 + 2*y(6)^2 + 2*z(6)^2 + 4*x(4)*x(8) + 4*x(5)*x(7) - 4*x(4)*xf + 4*y(4)*y(8) + 4*y(5)*y(7) - 4*y(4)*yf + 4*z(4)*z(8) + 4*z(5)*z(7) - 4*z(4)*zf;
%         P(16) = 3*x(5)*x(8) + 3*x(6)*x(7) - 3*x(5)*xf + 3*y(5)*y(8) + 3*y(6)*y(7) - 3*y(5)*yf + 3*z(5)*z(8) + 3*z(6)*z(7) - 3*z(5)*zf;
%         P(17) = x(7)^2 + y(7)^2 + z(7)^2 + 2*x(6)*x(8) - 2*x(6)*xf + 2*y(6)*y(8) - 2*y(6)*yf + 2*z(6)*z(8) - 2*z(6)*zf;
%         P(18) = x(7)*x(8) - x(7)*xf + y(7)*y(8) - y(7)*yf + z(7)*z(8) - z(7)*zf;
%         
%         if(abs(P(5)) <= eps)
%             P(5) = eps;
%         end
%         
%     end
%     
%     
%     if(traj_degree == 9)
%         
%         no_of_points = 19;
%         
%         P(1) = 9*x(1)^2 + 9*y(1)^2 + 9*z(1)^2;
%         P(2) = 17*x(1)*x(2) + 17*y(1)*y(2) + 17*z(1)*z(2);
%         P(3) = 8*x(2)^2 + 8*y(2)^2 + 8*z(2)^2 + 16*x(1)*x(3) + 16*y(1)*y(3) + 16*z(1)*z(3);
%         P(4) = 15*x(1)*x(4) + 15*x(2)*x(3) + 15*y(1)*y(4) + 15*y(2)*y(3) + 15*z(1)*z(4) + 15*z(2)*z(3);
%         P(5) =  7*x(3)^2 + 7*y(3)^2 + 7*z(3)^2 + 14*x(1)*x(5) + 14*x(2)*x(4) + 14*y(1)*y(5) + 14*y(2)*y(4) + 14*z(1)*z(5) + 14*z(2)*z(4);
%         P(6) = 13*x(1)*x(6) + 13*x(2)*x(5) + 13*x(3)*x(4) + 13*y(1)*y(6) + 13*y(2)*y(5) + 13*y(3)*y(4) + 13*z(1)*z(6) + 13*z(2)*z(5) + 13*z(3)*z(4);
%         P(7) = 6*x(4)^2 + 6*y(4)^2 + 6*z(4)^2 + 12*x(1)*x(7) + 12*x(2)*x(6) + 12*x(3)*x(5) + 12*y(1)*y(7) + 12*y(2)*y(6) + 12*y(3)*y(5) + 12*z(1)*z(7) + 12*z(2)*z(6) + 12*z(3)*z(5);
%         P(8) = 11*x(1)*x(8) + 11*x(2)*x(7) + 11*x(3)*x(6) + 11*x(4)*x(5) + 11*y(1)*y(8) + 11*y(2)*y(7) + 11*y(3)*y(6) + 11*y(4)*y(5) + 11*z(1)*z(8) + 11*z(2)*z(7) + 11*z(3)*z(6) + 11*z(4)*z(5);
%         P(9) =  5*x(5)^2 + 5*y(5)^2 + 5*z(5)^2 + 10*x(1)*x(9) + 10*x(2)*x(8) + 10*x(3)*x(7) + 10*x(4)*x(6) + 10*y(1)*y(9) + 10*y(2)*y(8) + 10*y(3)*y(7) + 10*y(4)*y(6) + 10*z(1)*z(9) + 10*z(2)*z(8) + 10*z(3)*z(7) + 10*z(4)*z(6);
%         P(10) = 9*x(1)*x(10) + 9*x(2)*x(9) + 9*x(3)*x(8) + 9*x(4)*x(7) + 9*x(5)*x(6) - 9*x(1)*xf + 9*y(1)*y(10) + 9*y(2)*y(9) + 9*y(3)*y(8) + 9*y(4)*y(7) + 9*y(5)*y(6) - 9*y(1)*yf + 9*z(1)*z(10) + 9*z(2)*z(9) + 9*z(3)*z(8) + 9*z(4)*z(7) + 9*z(5)*z(6) - 9*z(1)*zf;
%         P(11) = 4*x(6)^2 + 4*y(6)^2 + 4*z(6)^2 + 8*x(2)*x(10) + 8*x(3)*x(9) + 8*x(4)*x(8) + 8*x(5)*x(7) - 8*x(2)*xf + 8*y(2)*y(10) + 8*y(3)*y(9) + 8*y(4)*y(8) + 8*y(5)*y(7) - 8*y(2)*yf + 8*z(2)*z(10) + 8*z(3)*z(9) + 8*z(4)*z(8) + 8*z(5)*z(7) - 8*z(2)*zf;
%         P(12) = 7*x(3)*x(10) + 7*x(4)*x(9) + 7*x(5)*x(8) + 7*x(6)*x(7) - 7*x(3)*xf + 7*y(3)*y(10) + 7*y(4)*y(9) + 7*y(5)*y(8) + 7*y(6)*y(7) - 7*y(3)*yf + 7*z(3)*z(10) + 7*z(4)*z(9) + 7*z(5)*z(8) + 7*z(6)*z(7) - 7*z(3)*zf;
%         P(13) = 3*x(7)^2 + 3*y(7)^2 + 3*z(7)^2 + 6*x(4)*x(10) + 6*x(5)*x(9) + 6*x(6)*x(8) - 6*x(4)*xf + 6*y(4)*y(10) + 6*y(5)*y(9) + 6*y(6)*y(8) - 6*y(4)*yf + 6*z(4)*z(10) + 6*z(5)*z(9) + 6*z(6)*z(8) - 6*z(4)*zf;
%         P(14) = 5*x(5)*x(10) + 5*x(6)*x(9) + 5*x(7)*x(8) - 5*x(5)*xf + 5*y(5)*y(10) + 5*y(6)*y(9) + 5*y(7)*y(8) - 5*y(5)*yf + 5*z(5)*z(10) + 5*z(6)*z(9) + 5*z(7)*z(8) - 5*z(5)*zf;
%         P(15) = 2*x(8)^2 + 2*y(8)^2 + 2*z(8)^2 + 4*x(6)*x(10) + 4*x(7)*x(9) - 4*x(6)*xf + 4*y(6)*y(10) + 4*y(7)*y(9) - 4*y(6)*yf + 4*z(6)*z(10) + 4*z(7)*z(9) - 4*z(6)*zf;
%         P(16) = 3*x(7)*x(10) + 3*x(8)*x(9) - 3*x(7)*xf + 3*y(7)*y(10) + 3*y(8)*y(9) - 3*y(7)*yf + 3*z(7)*z(10) + 3*z(8)*z(9) - 3*z(7)*zf;
%         P(17) = x(9)^2 + y(9)^2 + z(9)^2 + 2*x(8)*x(10) - 2*x(8)*xf + 2*y(8)*y(10) - 2*y(8)*yf + 2*z(8)*z(10) - 2*z(8)*zf;
%         P(18) = x(9)*x(10) - x(9)*xf + y(9)*y(10) - y(9)*yf + z(9)*z(10) - z(9)*zf;
%         
%         if(abs(P(1)) <= eps)
%             P(1) = eps;
%         end
%         
%     end
    
    
    %P  = [A B C D E F]; % Polynomial of 5-th degree
    ti = roots(P); % calculate the roots
    
    T = zeros(no_of_points,1);
    T(no_of_points) = 1;
    
    % The solution tm of the polynomial with tm = [0,1] must be chosen,
    % and when this solution overtakes 1, k must be incremented
    % to change to a new spline segment.
    
    for i=1:(no_of_points-2)
        real_ti  = real(ti(i));
        angle_ti = angle(ti(i));
        
        % get only real postive roots tm = [0,1]
        if((real_ti>=0) && (real_ti<=1) && (angle_ti<1e-3) && (angle_ti>-1e-3))
            T(i) = real(ti(i));
        end
    end
    
    %% trajGetError
    
    % Calculate x-positon with t
    px = polyVal(traj_section.pos_x, T');
    
    % Calculate y-positon with t
    py = polyVal(traj_section.pos_y, T');
    
    % Calculate z-positon with t
    pz = polyVal(traj_section.pos_z, T');
    
    % return position vector
    curr_pos = [px; py; pz];
    
    for j = 1:no_of_points
        curr_pos(:,j) = curr_pos(:,j) - position;
    end
    
    error_array = vecnorm(curr_pos);
    
    [error, idx] = min(error_array);
    
    %%
    
    %[error, idx] = trajGetError(traj, T, position, k*ones(1,no_of_points));
    
    section_errors(k) = error;
    section_t(k)      = T(idx);
    
end

if(traj.num_sections_set > 0)
    [min_error, idx] = min(section_errors);
    
    section_idx = idx;
    error = min_error;
    t = section_t(idx);
else
    section_idx = 1;
    error = zeros(1,superiorfloat(position));
    t=zeros(1,superiorfloat(position));
end

end