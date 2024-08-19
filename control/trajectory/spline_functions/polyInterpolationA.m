function [A] = ...
    polyInterpolationA(points, degree, cycle)
% polyInterpolationA computes the A matrix for solving stepwise 
%   interpolated functions in 1D with Ax=b. The function calculates 
%   stepwise defined polynomials for any degree. The values are given are
%   assumed to be equally spaced with a constant stepsize.
%
%   Note:
%   This is a very specialized function for polynominal interpolation.
%   This function is not intended to replace a standard cubic spline
%   iterpolation, because the x values can not be choosen.
%
%   Known limitations:
%   - Do not use the cycle option and an even number of degree
%   - Very high grades lead to a very ill-conditioned system of equations
%
% Inputs:
%
%   points          y-coordinate of points to interpolate with step size of
%                   one(dimensionsless) each
%                   (1xN vector), dimensionsless
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
%   A               matrix A of equation of A*x=b
%
% Syntax:
%   [b] = polyInterpolationAx(points, degree, cycle)
%
% See also: polyInterpolationb, polyInterpolationAx,
%           polyInterpolationIterative

% Disclaimer:
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
A = zeros(size_A_mat, size_A_mat,superiorfloat(points,degree));

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
        
else
    
    % Boundary Condition Size
    bnd_left(:)  = 2;
    
    last_row = sub_mat_size*(num_of_splines-1)+bnd_left;
    last_right = sub_mat_size*(num_of_splines-1)+1;
    
    %Boundary for First Point and Last Point
    i = 1;
    j = 1;
    A(i  , j:j+degree) = point_0(1,:);
    
    A(i+1, last_right:last_right+degree) = point_1(1,:);
      
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

end
end