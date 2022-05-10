function [coeffs, num_of_splines, degree] = ...
    polyInterpolation(points, degree, cycle, plot_enable, derivatives)
% polyInterpolation computes stepwise interpolated functions in 1D
%   The function calculates stepwise defined polynomials for any degree.
%   The values are given are assumed to be equally spaced with a constant
%   stepsize.
%
%   Note:
%   This is a very specialized function for polynominal interpolation.
%   This function is not intended to replace a standard cubic spline
%   iterpolation, because the x values can not be choosen.
%
%   Known limitations:
%   - Do not use the cycle option and an even number
%   - Very high grades lead to a very ill-conditioned system of equations
%
% Inputs:
%
%   points          y-coordinate of points to interpolate with step size of
%                   one(dimensionsless) each
%                   (1xN vector)
%
%   degree 		    degree of the stepwise polynomials to calculate. This
%                   value should be an odd number to ensure symmetrically
%                   boundary conditions on every knot point (1,3,5,...)
%                   (scalar), dimensionless
%
%   cycle           enable automatic repetition of the given course
%                   to ensure that the derivatives of the first and last
%                   point are the same.
%                   (boolean)
%
%   plot_enable     Enable the plotting of the calculated functions
%                   (boolean)
%
%   derivatives     Count of the derivatives that will be plotted if 
%                   plotting is enabled.
%                   (boolean)
%
% Outputs:
%
%   traj            trajectory struct, see trajInit
%
% Syntax:
%   [traj] = trajFromWaypoints(traj, points, degree, cycle)
%
% See also: trajInit, traj_from_waypoints_example

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Fabian Guecker
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

if cycle == true
   points_new = [points, points(1)];
else
   points_new = points;
end

num_of_waypoints = length(points_new);
num_of_splines = num_of_waypoints-1;

pp = ones(degree+1, degree+1);
pp(2:end,:) = 0;
point_1 = pp;
for i=1:degree
    point_1(i+1,1:end-i) = polyder(point_1(i,1:end-i+1));
end

point_0 = point_1;
for i=1:degree
    point_0(i,1:end-i) = 0;
end

sub_mat_size = degree+1;
bnd_med = degree;
intermediate_size = sub_mat_size*(num_of_splines-1);

size_A_mat = sub_mat_size*num_of_splines;
A = zeros(size_A_mat, size_A_mat,'single');
b = zeros(size_A_mat, 1,'single');

if cycle == false
    
    % Boundary Condition Size
    bnd_left  = ceil((degree+1)/2);
    bnd_right = floor((degree+1)/2);
    
    %Left Boundary
    i = 1;
    j = 1;
    A(i:i+bnd_left-1, j:j+degree) = point_0(1:bnd_left,:);
    
    % Right Boundary
    i = bnd_left+1+intermediate_size;
    j = 1+intermediate_size;
    A(i:i+bnd_right-1, j:j+degree) = point_1(1:bnd_right,:);
    
    % Construct Right Hand Side Boundarys
    b(1) = points_new(1);
    b(end-bnd_right+1) = points_new(1,end);
    
    % 1st Derivative should not be zero at endpoints
    % this cause problems with newton iteration
    if(degree>1)
        b(2) = points_new(2)-points_new(1);
    end
    if(degree>2)
        b(end-bnd_right+2) = points_new(end)-points_new(end-1);
    end
        
else
    
    % Boundary Condition Size
    bnd_left(:)  = 2;
    bnd_right = floor((degree+1)/2);
    
    last_row = sub_mat_size*(num_of_splines-1)+bnd_left;
    last_right = sub_mat_size*(num_of_splines-1)+1;
    
    %Boundary for First Point and Last Point
    i = 1;
    j = 1;
    A(i  , j:j+degree) = point_0(1,:);
    A(i+1, last_right:last_right+degree) = point_1(1,:);

    % Construct Right Hand Side Boundarys
    b(1) = points_new(1,1);   %These should be the same!
    b(2) = points_new(1,end); %These should be the same!
      
    % Derivative of Startpoint First Segment
    i = last_row+1;
    j = 1;
    A(i:i+bnd_med-2, j:j+degree) = point_0(2:bnd_med,:);
    
    % Derivative of Endpoint Last Segment
    i = last_row+1;
    j = last_right;
    A(i:i+bnd_med-2, j:j+degree) = -point_1(2:bnd_med,:);
    
end

for k = 1:(num_of_splines-1)
    
    itm_row = sub_mat_size*(k-1)+bnd_left;
    itm_left = sub_mat_size*(k-1)+1;
    itm_right = itm_left+sub_mat_size;
    
    % Intermediate Step, Endpoint Left Segment
    i = itm_row+1;
    j = itm_left;
    A(i, j:j+degree) = point_1(1,:);
    
    % Intermediate Step, Startpoint Right Segment
    i = itm_row+2;
    j = itm_right;
    A(i, j:j+degree) = point_0(1,:);
    
    % Derivative of Endpoint Left Segment
    i = itm_row+3;
    j = itm_left;
    A(i:i+bnd_med-2, j:j+degree) = point_1(2:bnd_med,:);
    
    % Derivative of Endpoint Right Segment
    i = itm_row+3;
    j = itm_right;
    A(i:i+bnd_med-2, j:j+degree) = -point_0(2:bnd_med,:);
    
    % Construct Right Hand Side Boundarys
    b(itm_row+1) = points_new(1,k+1);
    b(itm_row+2) = points_new(1,k+1);
end
    
% Solve the System
x = A \ b;
coeffs = full(x)';
%% Plot function and derivatives
if(plot_enable)
    
    figure(1)
    clf;
    subplot(derivatives+1, 1, 1)
    hold on
    
    for i=1:num_of_splines
        idx_beg = sub_mat_size*(i-1)+1;
        idx_end = sub_mat_size*(i);
        x_iter  = coeffs(idx_beg:idx_end);
        plot(i-1:0.01:i,polyval(x_iter,0:0.01:1))
    end
    hold off
    grid on
    
    
    for k = 1:derivatives
        subplot(derivatives+1, 1, k+1)
        
        hold on
        for i=1:num_of_splines
            idx_beg = sub_mat_size*(i-1)+1;
            idx_end = sub_mat_size*(i);
            x_iter  = coeffs(idx_beg:idx_end);
            
            x_derivative = x_iter;
            for deriv_no=1:k
                x_derivative = polyder(x_derivative);
            end
            
            plot(i-1:0.01:i,polyval(x_derivative,0:0.01:1))
        end
        hold off
        grid on
        
    end
end

end