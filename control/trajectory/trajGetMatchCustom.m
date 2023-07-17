function [section_idx, error, t] = ... 
    trajGetMatchCustom(traj, position, active_section, R_turn, T_vec)
% trajGetMatchCustom computes the nearest point on the trajectory
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
%   [section_idx, error, t] = trajGetMatchCustom(traj, position, active_section)
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

%traj_section = trajGetSection(traj, 1);

t = zeros(1,superiorfloat(position));
section_idx = ones(1,superiorfloat(position));
error = cast(1e16, 'like', t);

for k = first_checked_section:last_checked_section
    
    traj_section = traj.sections(k);    
       
%     R_turn = 10;
%     T_vec = [1; 0; 0];
%     
    fun = @(t) trajCalcDistance(traj_section, R_turn, position, T_vec, t, false);
%     fun = @(t) (polyVal(traj_section.pos_x, t) - position(1))^2 + (polyVal(traj_section.pos_y, t) - position(2))^2 + (polyVal(traj_section.pos_z, t) - position(3))^2;

%     figure(16)
%     clf;
%     plot(t_array, error_array)
%     
%     hold on
    [t_min, ~, ~] = polyFindMin(fun, 0, 1);
%     hold off    

%     disp(x_final)

    curr_error = trajCalcDistance(traj_section, R_turn, position, T_vec, t_min, false);
    
    if( curr_error < error)
        t           = cast(t_min,'like',t);
        error       = curr_error;       
        section_idx = k;
    end
    
end

end

 