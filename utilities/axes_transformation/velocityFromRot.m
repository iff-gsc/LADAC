%velocityFromRot    computes the velocity of a point on a rotating body
% This function computes the velocity of a point p fixed to frame x
% relative to frame y. It is the sum of the velocity of frame x relative to
% frame y and the cross product of the angular velocity of frame x relative
% frame y times the position of point p represented in frame x [1, p. 
% 20-21].
% 
% Literature:
% [1]   Stevens, B. L. et al. (2016): Aircraft Control and Simulation.
%       Dynamics, Control Design, and Autonomous Systems. 3rd ed. Wiley.
%
% Inputs:

%   V_xy_x      the velocity vector of frame x relative to frame y
%               represented in frame x, in m/s
%   omega_xy_x  the angular velocity vector of frame x relative to frame y
%               represented in frame x, in rad/s
%   r_p_x       the position vector of a point p represented in frame x, 
%               in m 
%
% Outputs:
%   V_py_x      the velocity vector of point p relative to frame y
%               represented in frame x, in m/s
%
% See also: forceTransform

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

function V_py_x = velocityFromRot( V_xy_x, omega_xy_x, r_p_x )%#codegen

% determine number of input vectors
num_vectors = size( r_p_x, 2 );
% computation of the velocity of point p relative to frame y represented in
% frame x [1, p. 21]
V_py_x = repmat(V_xy_x,1,num_vectors) + crossFast( repmat(omega_xy_x,1,num_vectors), r_p_x );

end