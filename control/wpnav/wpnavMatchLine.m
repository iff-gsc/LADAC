function [p_match,t,d] = wpnavMatchLine( p1, p2, p )
% wpnavMatchLine match the current position to the line segment
%   This function performs the matching of a given current point p to the
%   specified line segments via the points p1 and p2.
% 
% Syntax:
%   [p_match,t,d] = wpnavMatchLine( p1, p2, p )
% 
% Inputs:
%   p1                  Position of first point (3x1 array) in g frame
%                       (north-east-down), in m
%   p2                  Position of second point (3x1 array) in g frame
%                       (north-east-down), in m
%   p                   Current position (3x1 array) in g frame
%                       (north-east-down), in m
% 
% Outputs:
%   p_match             Matched position (3x1 array) on line segment or
%                       circle segment on the waypoint track in g frame
%                       (north-east-down), in m
%   t                   Non-dimensional time (scalar): 0 at the start of
%                       the segment, 1 at the end of the segment
%   d                   Position error (scalar): distance from the current
%                       position to the matched position, in m
% 
% See also:
%   wpnavMatch, wpnavMatchCircSeg

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2024 Yannic Beyer
%   Copyright (C) 2024 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

b = p2-p1;

denom = dot(b,b);
if denom < 1
    denom(:) = 1;
end

t = divideFinite( dot(p-p1,b), denom );

p_match = wpnavLineGetPos(p1,p2,t);
d = norm( p_match - p, 2);

end