function n_g_des = ndiCopterAcc2LeanVector( a_g, g )
% ndiCopterAcc2LeanVector desired acceleration to lean vector with NDI
%   For multicopters where the thrust vector points always in negative
%   body-fixed z direction, the desired acceleration can be achieved by
%   tilting the thrust vector (vehicle aerodynamics like drag are
%   neglected). The reduced attitude concept is described in [2].
% 
% Syntax:
%   n_g = ndiCopterAcc2LeanVector( a_g, g )
% 
% Inputs:
%   a_g             Desired acceleration excluding gravity (3x1 array) in
%                   geodetic (g) frame, in m/s^2
%   g               gravitational acceleration (scalar), in m/s^2
% 
% Outputs:
%   n_g_des         desired lean vector as input for the attitude
%                   controller cascade in geodetic (g) frame; the lean
%                   vector is used for reduced attitude control, it points
%                   into the direction of the thrust vector and is a unit
%                   vector (3x1 array), see [1], section II.B.,
%                   dimensionless
% 
% Literature:
%   [1] Sun, S., Wang, X., Chu, Q., & de Visser, C. (2020). Incremental
%       nonlinear fault-tolerant control of a quadrotor with complete loss
%       of two opposing rotors. IEEE Transactions on Robotics, 37(1),
%       116-130.
%   [2] Brescianini, D., & D’Andrea, R. (2018). Tilt-prioritized
%       quadrocopter attitude control. IEEE Transactions on Control Systems
%       Technology, 28(2), 376-387.
% 
% See also:
%   indiCopterAcc2LeanVector, quatReduced, dcm2LeanVector

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% convert acceleration to lean vector, see [1], eq. (24)
g_g = [0;0;g];
n_g_des = ( a_g - g_g ) / norm( a_g - g_g, 2 );

end