function s_g = wpnavCircSegGetPos( circ_seg, t )
% wpnavCircSegGetPos get position on circle segment
% 
% Syntax:
%   s_g = wpnavCircSegGetPos( circ_seg, t )
% 
% Inputs:
%   circ_seg            Circle segment (struct as defined by wpnavCircSeg)
%   t                   Non-dimensional time on circle segment: 0 is at the
%                       start and 1 is at the end
% 
% Outputs:
%   s_g                 Position (3x1 array) on circle segment in g frame
%                       (north-east-down), in m
% 
% See also:
%   wpnavCircSeg, wpnavMatch, wpnavCircSegGetAcc, wpnavCircSegGetVel

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2024 Yannic Beyer
%   Copyright (C) 2024 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

angle = t * circ_seg.angle;
s_g = circ_seg.center + axisAngle(circ_seg.start-circ_seg.center,circ_seg.n,angle);

end