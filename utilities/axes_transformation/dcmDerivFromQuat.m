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

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

function M_bg_dt = dcmDerivFromQuat(q_bg,q_bg_dt)

% normalize quaternion
q_bg = quatNormalize( q_bg );

% compute all multiplications (multiple use)
q0_q0_dt = 2*q_bg(1)*q_bg_dt(1);
q1_q1_dt = 2*q_bg(2)*q_bg_dt(2);
q2_q2_dt = 2*q_bg(3)*q_bg_dt(3);
q3_q3_dt = 2*q_bg(4)*q_bg_dt(4);
q0_q1_dt = q_bg_dt(1)*q_bg(2) + q_bg(1)*q_bg_dt(2);
q0_q2_dt = q_bg_dt(1)*q_bg(3) + q_bg(1)*q_bg_dt(3);
q0_q3_dt = q_bg_dt(1)*q_bg(4) + q_bg(1)*q_bg_dt(4);
q1_q2_dt = q_bg_dt(2)*q_bg(3) + q_bg(2)*q_bg_dt(3);
q1_q3_dt = q_bg_dt(2)*q_bg(4) + q_bg(2)*q_bg_dt(4);
q2_q3_dt = q_bg_dt(3)*q_bg(4) + q_bg(3)*q_bg_dt(4);

% derivative of DCM w.r.t. time.
% DCM according to [1, page 53]
M_bg_dt = [ 
    q0_q0_dt + q1_q1_dt - q2_q2_dt - q3_q3_dt,...
    2*(q1_q2_dt + q0_q3_dt),...
    2*(q1_q3_dt - q0_q2_dt); ...
    ...
    2*(q1_q2_dt - q0_q3_dt),...
    q0_q0_dt - q1_q1_dt + q2_q2_dt - q3_q3_dt,...
    2*(q2_q3_dt + q0_q1_dt);
    ...
    2*(q1_q3_dt + q0_q2_dt),...
    2*(q2_q3_dt - q0_q1_dt),...
    q0_q0_dt - q1_q1_dt - q2_q2_dt + q3_q3_dt ];
end
