function [b] = polyInterpolationb(points, degree, cycle)
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
% Outputs:
%
%   b               right-hand-side b if equation of A*x=b
%
% Syntax:
%   [b] = polyInterpolationb(points, degree, cycle)
%
% See also: polyInterpolationb, polyInterpolationAx,
%           polyInterpolationIterative

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

size_A_mat = sub_mat_size*num_of_splines;
%b = zeros(size_A_mat, 1,superiorfloat(points,degree));
b = zeros(size_A_mat, 1);

if cycle == false
    
    % Boundary Condition Size
    bnd_left  = ceil((degree+1)/2);
    bnd_right = floor((degree+1)/2);
        
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
    bnd_left  = 2;

    % Construct Right Hand Side Boundarys
    b(1) = points_new(1,1);   %These should be the same!
    b(2) = points_new(1,end); %These should be the same!
    
end

for k = 1:(num_of_splines-1)
    
    itm_row = sub_mat_size*(k-1)+bnd_left;
    
    % Construct Right Hand Side Boundarys
    b(itm_row+1) = points_new(1,k+1);
    b(itm_row+2) = points_new(1,k+1);
end
   
end