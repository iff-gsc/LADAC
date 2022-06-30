% quat2Euler computes the Euler angles (Phi) from quaternions.
%   The Euler angles are used to represent the attitude of the body
%   relative to the earth. However, because of the gimbal lock phenomena
%   they are rarely used as states. Instead, quaternions are used. The
%   Euler angles can be computes from quaternions anytime.
% 
% Literature:
% [1]   Stevens, B. L. et al. (2016): Aircraft Control and Simulation.
%       Dynamics, Control Design, and Autonomous Systems. 3rd ed. Wiley.
% 
% Inputs:
%   q_bg            the four dimensional norm quaternion vector from
%                   north-east-down (NED) frame to body-fixed frame
% 
% Outputs:
%   EulerAngles     vector with 3 elements containing the Euler angles Phi,
%                   Theta and Psi, rad
% 
% 
% See also: euler2Quat
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

function EulerAngles  = quat2Euler( q_bg ) %#codegen

% normalize quaternion
q_bg = quatNormalize( q_bg );

% extract components of vector
q0 = q_bg(1); q1 = q_bg(2); q2 = q_bg(3); q3 = q_bg(4);

q02 = powerFast(q0,2);
q12 = powerFast(q1,2);
q22 = powerFast(q2,2);
q32 = powerFast(q3,2);

% compute the required components of the DCM according to [1, page 12]
c_23 = 2 * ( q2 * q3 + q0 * q1 );
c_33 = q02 - q12 - q22 + q32;
c_13 = 2 * ( q1 * q3 - q0 * q2 );
c_12 = 2 * ( q1 * q2 + q0 * q3 );
c_11 = q02 + q12 - q22 - q32;

% compute the Euler angles according to [1, page 56] and [1, page 12]
Phi = atan2( c_23, c_33 );
Theta = - asinReal(c_13);
Psi = atan2( c_12, c_11 );

% build the vector of Euler angles
EulerAngles = [ Phi; Theta; Psi ];

end