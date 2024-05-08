function s_g = wpnavLineGetPos( p1 ,p2, t )
% wpnavLineGetPos get position on line segment (between two points)
% 
% Syntax:
%   s_g = wpnavLineGetPos( p1 ,p2, t )
% 
% Inputs:
%   p1                  Position of first point (3x1 array) in g frame
%                       (north-east-down), in m
%   p2                  Position of second point (3x1 array) in g frame
%                       (north-east-down), in m
%   t                   Non-dimensional time on line segment: 0 is at the
%                       start (p1) and 1 is at the end (p2)
% 
% Outputs:
%   s_g                 Position (3x1 array) on line segment in g frame
%                       (north-east-down), in m
% 
% See also:
%   wpnavMatchLine, wpnavMatch, wpnavLineGetVel

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2024 Yannic Beyer
%   Copyright (C) 2024 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

b = p2 - p1;
s_g = p1 + t*b;

end