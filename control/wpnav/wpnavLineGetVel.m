function V_Kg = wpnavLineGetVel( p1, p2, V )
% wpnavLineGetVel get velocity on line segment (between two points)
% 
% Syntax:
%   V_Kg = wpnavLineGetVel( p1, p2, V )
% 
% Inputs:
%   p1                  Position of first point (3x1 array) in g frame
%                       (north-east-down), in m
%   p2                  Position of second point (3x1 array) in g frame
%                       (north-east-down), in m
%   V                   Velocity (scalar) along the line segment, in m/s
% 
% Outputs:
%   V_Kg                Velocity (3x1 array) on line segment in g frame
%                       (north-east-down), in m
% 
% See also:
%   wpnavMatchLine, wpnavMatch, wpnavLineGetPos

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2024 Yannic Beyer
%   Copyright (C) 2024 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

dir_vec_unit = p2-p1;
dir_vec_unit = divideFinite( dir_vec_unit, norm( dir_vec_unit, 2 ) );
V_Kg = V * dir_vec_unit;

end