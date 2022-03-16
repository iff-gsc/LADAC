function [q_red,yaw] = quatReduced(q_bg)
% quatReduced compute reduced attitude quaternion and yaw rotation angle
%   For multicopters the concept of the so called reduced attitude was
%   developed for tilt-prioritized attitude control [1].
% 
% Synatx:
%   [q_red,yaw] = quatReduced(q_bg)
% 
% Inputs:
%   q_bg        quaternion (4x1 array) from frame g to frame b,
%               dimensionless
% 
% Outputs:
%   q_red       reduced attitude quaternion (4x1 array), dimensionless
%   yaw         yaw rotation angle (scalar), in rad
% 
% See also:
%   quatInv, quatMultiply
% 
% Literature:
%   [1] Brescianini, D., & D�Andrea, R. (2018). Tilt-prioritized
%       quadrocopter attitude control. IEEE Transactions on Control Systems
%       Technology, 28(2), 376-387.
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Fabian Guecker
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% quaternion to DCM
M_bg = quat2Dcm( q_bg );

% lean angles
[phi,delta] = dcm2Lean( M_bg );

% reduced attitude quaternion
% [1], eq. (1)
k = [ sin(delta); -cos(delta); 0 ];
q_red = [ ...
    cos(phi/2); ...
    k * sin(phi/2); ...
    ];

q_yaw = quatMultiply( quatInv(q_red), q_bg );

% [1], eq. (20) and eq. (1)
q_yaw = quatNormalize(q_yaw);
yaw = 2*acosReal( sign(q_yaw(4)) * q_yaw(1) );

end