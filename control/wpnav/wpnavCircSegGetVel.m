function V_Kg = wpnavCircSegGetVel( circ_seg ,t ,V )
% wpnavCircSegGetVel get tangential velocity vector on circle segment
% 
% Syntax:
%   V_Kg = wpnavCircSegGetVel( circ_seg ,t ,V )
% 
% Inputs:
%   circ_seg            Circle segment (struct as defined by wpnavCircSeg)
%   t                   Non-dimensional time on circle segment: 0 is at the
%                       start and 1 is at the end
%   V                   Velocity along the circle segment (scalar), in m/s
% 
% Outputs:
%   V_Kg                Tangential velocity (3x1 array) on circle segment
%                       in g frame (north-east-down), in m/s
% 
% See also:
%   wpnavCircSeg, wpnavMatch, wpnavCircSegGetPos, wpnavCircSegGetVel

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2024 Yannic Beyer
%   Copyright (C) 2024 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

tangent_vec_p1_unit = cross( circ_seg.n, circ_seg.start-circ_seg.center );
tangent_vec_p1_unit = divideFinite( tangent_vec_p1_unit, norm( tangent_vec_p1_unit, 2 ) );

angle = t * circ_seg.angle;
tangent_vec_t_unit = axisAngle(tangent_vec_p1_unit,circ_seg.n,angle);

V_Kg = tangent_vec_t_unit * V;

end