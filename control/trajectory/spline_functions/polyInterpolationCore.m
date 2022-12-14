function [x, i] = polyInterpolationCore(points, degree, cycle)

polyInterpolation computes stepwise interpolated functions in 1D
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
%   - Do not use the cycle option and an even number of degree
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
%   x            trajectory struct, see trajInit
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

[b, num_of_splines] = polyInterpolationb(points, degree, cycle);
A = @(x, a) polyInterpolationAx(num_of_splines, degree, cycle, x, a);
[ x, w, u, v, Anorm, alfa, rhobar, phibar ] = ladacLsqrInit(A, b);

for i=1:1000
    [ x, w, u, v, Anorm, alfa, rhobar, phibar ] = ...
        ladacLsqrIterate( A, x, w, u, v, Anorm, alfa, rhobar, phibar);
    if( norm( A(x,1) - b ) < 1e-4)
        disp(i);
        break;
        
    end
end


end