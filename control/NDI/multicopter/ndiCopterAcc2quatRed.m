function q_red = ndiCopterAcc2quatRed( a_g, g )
% ndiCopterAcc2quatRed desired acceleration to reduced attitude quaternion (NDI)
%   For multicopters where the thrust vector points always in negative
%   body-fixed z direction, the desired acceleration can be achieved by
%   tilting the thrust vector (vehicle aerodynamics like drag are
%   neglected). The reduced attitude concept is described in [1].
% 
% Syntax:
%   q_red = ndiCopterAcc2quatRed( a_g, g )
% 
% Inputs:
%   a_g             Desired acceleration excluding gravity (3x1 array) in
%                   geodetic (g) frame, in m/s^2
%   g               gravitational acceleration (scalar), in m/s^2
% 
% Outputs:
%   q_red           reduced attitude quaternion from body-fixed frame to
%                   geodetic frame (4x1 array), dimensionless
% 
% Literature:
%   [1] Brescianini, D., & D’Andrea, R. (2018). Tilt-prioritized
%       quadrocopter attitude control. IEEE Transactions on Control Systems
%       Technology, 28(2), 376-387.
% 
% See also:
%   indiCopterAcc2LeanVector, quatReduced

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

a_g_xy = sqrtReal( powerFast(a_g(1),2) + powerFast(a_g(2),2) );

% [1], eq. (50)
phi_red = atan2( a_g_xy, a_g(3) + g );
% [1], eq. (51)
n_g_red = divideFinite( [ -a_g(2); a_g(1); 0 ], a_g_xy );

phi_red_half = phi_red/2;

% [1], eq. (1)
q_red = [ cos( phi_red_half ); sin( phi_red_half ) * n_g_red ];

end