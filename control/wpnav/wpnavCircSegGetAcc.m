function a_Kg = wpnavCircSegGetAcc( circ_seg, t, V )
% wpnavCircSegGetAcc get required acceleration to follow circle segment
% 
% Syntax:
%   a_Kg = wpnavCircSegGetAcc( circ_seg, t, V )
% 
% Inputs:
%   circ_seg            Circle segment (struct as defined by wpnavCircSeg)
%   t                   Non-dimensional time on circle segment (scalar):
%                       0 is at the start and 1 is at the end
%   V                   Velocity (scalar) along the circle segment, in m/s
% 
% Outputs:
%   a_Kg                Required acceleration (3x1 array) to follow the
%                       circle segment in g frame (north-east-down) (does
%                       not include the gravitational acceleration),
%                       in m/s/s
% 
% See also:
%   wpnavCircSeg, wpnavMatch, wpnavCircSegGetPos, wpnavCircSegGetVel

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2024 Yannic Beyer
%   Copyright (C) 2024 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

p = wpnavCircSegGetPos( circ_seg, t );
dir_vec_unit = circ_seg.center - p;
dir_vec_unit = divideFinite( dir_vec_unit, norm( dir_vec_unit, 2 ) );
a_Kg = divideFinite( V*V, circ_seg.r ) * dir_vec_unit;

end