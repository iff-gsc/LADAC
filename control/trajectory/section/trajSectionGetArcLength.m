function [arc_length, arc_length_dt] = ...
trajSectionGetArcLength(traj_section, varargin)
% trajSectionGetArcLength returns the arc from a trajectory
%   The distance covered and the dimensionless time parameter t are 
%   non-linearly dependent on each other. The function returns the true 
%   arclength from the beginning of the section to a given t from [0-1].
%
% Inputs:
%   traj_section 	trajectory section struct, see trajSectionInit         
%
%   t               dimensionless time parameter 
%
% Outputs:
%   arc_length      arc length (scalar), in m
%   arc_length_dt   arc length derivative (scalar), in m/(dimensionsless t)
%
% Syntax:
%   [arc_length, arc_length_dt] = trajSectionGetArcLength(traj_section) 
%   [arc_length, arc_length_dt] = trajSectionGetArcLength(traj_section, t) 
%
% See also: trajSectionInit, trajSectionGetVel, trajSectionGetAcc
%

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Fabian Guecker
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************



if isempty(varargin)
    t = traj_section.t;
else
    t = real(varargin{1});
end    

% Protect the boundaries but leave room for Newton iteration directly on
% the boundary values
t = max(min(1.1, t), -0.1);

% Derivative of x position
dx = polyder(traj_section.pos_x);

% Derivative of y position
dy = polyder(traj_section.pos_y);

% Derivative of z position
dz = polyder(traj_section.pos_z);

% Function handle of the arc length derivative
arc_length_fun = @(ts) sqrt( polyval(dx, ts).^2 + ...
                             polyval(dy, ts).^2 + ...
                             polyval(dz, ts).^2);                       

% Numerical integration of the arc length derivative from [0, t]
%arc_length = 0.0;%quadgk(arc_length_fun, 0, t);
arc_length = integrateSimpson( arc_length_fun, 0, t, 15);

% Return the arc length derivative
arc_length_dt = arc_length_fun(t);

end


function [ sum ] = integrateSimpson( func, left, right, steps)
t = left; 
sum = 0;
step = (right-left) / steps;
step_1_2 = 0.5*step;

for i = 1:steps
    sum(:) = sum + (func(t) + 4.0 * func(t + step_1_2) + func(t + step));   
    t(:) = t + step;
end

sum(:) = (1/6) * step * sum;

end
