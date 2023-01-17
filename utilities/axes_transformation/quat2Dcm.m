% dcm2Euler computes the Euler angles (Phi) from the direction cosine matrix
% (DCM)
%   The Euler angles are used to represent the attitude of the body
%   relative to the earth. They can be computed from the DCM.
% 
% Literature:
% [1]   Stevens, B. L. et al. (2016): Aircraft Control and Simulation.
%       Dynamics, Control Design, and Autonomous Systems. 3rd ed. Wiley.
% 
% Inputs:
%   q_bg            four dimensional quaternion from g frame to b frame
% 
% Outputs:
%   EulerAngles     vector with 3 elements containing the Euler angles Phi,
%                   Theta and Psi, rad
% 
% See also: euler2Dcm
%

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

function M_bg = quat2Dcm(q_bg) %#codegen

% normalize quaternion
q_bg = quatNormalize( q_bg );

% compute all multiplications (multiple use)
q0_q0 = q_bg(1)^2;
q1_q1 = q_bg(2)^2;
q2_q2 = q_bg(3)^2;
q3_q3 = q_bg(4)^2;
q0_q1 = q_bg(1)*q_bg(2);
q0_q2 = q_bg(1)*q_bg(3);
q0_q3 = q_bg(1)*q_bg(4);
q1_q2 = q_bg(2)*q_bg(3);
q1_q3 = q_bg(2)*q_bg(4);
q2_q3 = q_bg(3)*q_bg(4);

% compute the DCM according to [1, page 53]
M_bg = [ 
    q0_q0 + q1_q1 - q2_q2 - q3_q3,...
    2*(q1_q2 + q0_q3),...
    2*(q1_q3 - q0_q2); ...
    ...
    2*(q1_q2 - q0_q3),...
    q0_q0 - q1_q1 + q2_q2 - q3_q3,...
    2*(q2_q3 + q0_q1);
    ...
    2*(q1_q3 + q0_q2),...
    2*(q2_q3 - q0_q1),...
    q0_q0 - q1_q1 - q2_q2 + q3_q3 ];
end
