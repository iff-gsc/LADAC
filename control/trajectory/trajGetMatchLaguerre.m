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

resolution = 2*degree;
t_array = linspace(0, 1, resolution)*ones(1,superiorfloat(position));

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

% Polynomial coefficents
x = traj.sections(section_idx).pos_x';
y = traj.sections(section_idx).pos_y';
z = traj.sections(section_idx).pos_z';

xf = position(1);
yf = position(2);
zf = position(3);

% Calculate first derivative of the squared distance
P(1) = 5*x(1)^2 + 5*y(1)^2 + 5*z(1)^2;
P(2) = 9*x(1)*x(2) + 9*y(1)*y(2) + 9*z(1)*z(2);
P(3) = 4*x(2)^2 + 4*y(2)^2 + 4*z(2)^2 + 8*x(1)*x(3) + 8*y(1)*y(3) + 8*z(1)*z(3);
P(4) = 7*x(1)*x(4) + 7*x(2)*x(3) + 7*y(1)*y(4) + 7*y(2)*y(3) + 7*z(1)*z(4) + 7*z(2)*z(3);
P(5) = 3*x(3)^2 + 3*y(3)^2 + 3*z(3)^2 + 6*x(1)*x(5) + 6*x(2)*x(4) + 6*y(1)*y(5) + 6*y(2)*y(4) + 6*z(1)*z(5) + 6*z(2)*z(4);
P(6) = 5*x(1)*x(6) + 5*x(2)*x(5) + 5*x(3)*x(4) - 5*x(1)*xf + 5*y(1)*y(6) + 5*y(2)*y(5) + 5*y(3)*y(4) - 5*y(1)*yf + 5*z(1)*z(6) + 5*z(2)*z(5) + 5*z(3)*z(4) - 5*z(1)*zf;
P(7) = 2*x(4)^2 + 2*y(4)^2 + 2*z(4)^2 + 4*x(2)*x(6) + 4*x(3)*x(5) - 4*x(2)*xf + 4*y(2)*y(6) + 4*y(3)*y(5) - 4*y(2)*yf + 4*z(2)*z(6) + 4*z(3)*z(5) - 4*z(2)*zf;
P(8) = 3*x(3)*x(6) + 3*x(4)*x(5) - 3*x(3)*xf + 3*y(3)*y(6) + 3*y(4)*y(5) - 3*y(3)*yf + 3*z(3)*z(6) + 3*z(4)*z(5) - 3*z(3)*zf;
P(9) = x(5)^2 + y(5)^2 + z(5)^2 + 2*x(4)*x(6) - 2*x(4)*xf + 2*y(4)*y(6) - 2*y(4)*yf + 2*z(4)*z(6) - 2*z(4)*zf;
P(10) = x(5)*x(6) - x(5)*xf + y(5)*y(6) - y(5)*yf + z(5)*z(6) - z(5)*zf;

t = laguerre(P, t, 1e-16, 100);

t = min(max(t,0),1);

end