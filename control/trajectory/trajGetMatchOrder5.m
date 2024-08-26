function [section_idx, error_sqr, t] = trajGetMatchOrder5(traj, position, active_section)
% trajGetMatchOrder5 computes the nearest point on the trajectory
%   The function returns the point on the trajectory that has minimum
%   euclidean norm in respect to the given position.
%   This function only works with a polynominal degree of five.
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

if(traj.polynomial_degree ~= 5)
    error('Polynominal order is not 5.')
end

% return values
section_idx = ones(1,superiorfloat(position));
error_sqr = realmax(superiorfloat(position));
t = zeros(1,1,superiorfloat(position));

% aircraft's position
xf = position(1);
yf = position(2);
zf = position(3);

% Section Range
first_checked_section = 1;
last_checked_section = traj.num_sections_set;

% poly-vector
P = zeros(1,10,superiorfloat(position));

% Precalculation of subdiagonal matrix for schur decomposition
A5 = diag(ones(8,1,superiorfloat(position)),-1);

% Iterate through all relevant sections
for k = first_checked_section:last_checked_section
    
    % Polynomial coefficents
    x = traj.sections(k).pos_x';
    y = traj.sections(k).pos_y';
    z = traj.sections(k).pos_z';
         
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
    
    %% Solve eigenvalue problem with companion matrix to find the roots of P
    
    % make sure first entry is not zero (prevent division by zero)
    if(abs(P(1)) <= eps(P(1)))
        P(1) = eps(P(1));
    end
    
    % build the companion matrix
    A = A5;
    A(1,:) = -P(2:10)./P(1);
    
    % solve eigenvalue problem with schur decomposition
    Schur_mat = schur(A);
    
    % The diagonal elements hold the eigenvalues, each of them is a
    % root of P. It is a cadidate for the segments minimum if it is
    % real and satisfies the condition 0 <= t <= 1. Check also the both
    % edges of the segment.
    
    T = [Schur_mat(1,1); Schur_mat(2,2); Schur_mat(3,3);
        Schur_mat(4,4); Schur_mat(5,5); Schur_mat(6,6);
        Schur_mat(7,7); Schur_mat(8,8); Schur_mat(9,9);
        0; 1];
    
    %clc;
   % disp(k)
   % disp(T')
   % [u,v] = eig(A);
    

    
  %  [dominant_eigenvalue, dominant_eigenvector] = all_eigenvalues_power_method(A, 1e-6, 100)
    %% Calculate the squared error for every candidate between 0 and  1
    
    t_values = zeros(1,6,superiorfloat(position));
    curr_pos = zeros(1,3,superiorfloat(position));
    
    for i=1:11
        T_i = T(i);
        if (T_i <= 1.0) && (T_i >= 0.0)
            
            % Loop unrolling for calculation of t, t^2, t^3, ...
            t_values(6) = 1;
            t_values(5) = T_i;
            t_values(4) = t_values(5) * T_i;
            t_values(3) = t_values(4) * T_i;
            t_values(2) = t_values(3) * T_i;
            t_values(1) = t_values(2) * T_i;
            
            % calculate the distance in all three dimensions
            curr_pos(1) = t_values * x - xf;
            curr_pos(2) = t_values * y - yf;
            curr_pos(3) = t_values * z - zf;
            
            % calculate the squared error
            curr_error = sum(curr_pos .* curr_pos);
            
            % check if current error is smaller then the smallest found 
            if(curr_error < error_sqr)
                t = T_i;
                section_idx = k;
                error_sqr = curr_error;
            end
        end
    end
    
end

end