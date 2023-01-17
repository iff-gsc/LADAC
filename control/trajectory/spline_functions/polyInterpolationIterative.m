function [coeffs, num_of_splines, degree] = ...
    polyInterpolationIterative(points, degree, cycle, plot_enable, derivatives)
% polyInterpolationIterative computes stepwise interpolated functions in 1D
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
%                   (1xN vector), dimensionless
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
%   coeffs          solution of the system Ax=b
%                   (1xN vector), dimensionless
%
% Syntax:
%   [coeffs, num_of_splines, degree] = ...
%   polyInterpolationIterative(points, degree, cycle, plot_enable, ...
%   derivatives)
%
% See also: polyInterpolationb, polyInterpolationAx,
%           trajFromWaypointsIterative, polyInterpolationCore

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
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
sub_mat_size = degree+1;
    
%% Solve the System
tic
[x,iter] = polyInterpolationCore(points, degree, cycle);
toc

disp(iter)

coeffs = x';


%% Plot function and derivatives
if(plot_enable)
    
    clf;
    subplot(derivatives+1, 1, 1)
    hold on
    
    for i=1:num_of_splines
        idx_beg = sub_mat_size*(i-1)+1;
        idx_end = sub_mat_size*(i);
        x_iter  = coeffs(idx_beg:idx_end);
        plot(i-1:0.01:i,polyVal(x_iter,0:0.01:1))
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
            
            plot(i-1:0.01:i,polyVal(x_derivative,0:0.01:1))
        end
        hold off
        grid on
        
    end
end

end