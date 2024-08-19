% dcm2Quat computes the quaternion (q_bg) from the direction cosine matrix
% (DCM)
%   The quaternion is used to represent the attitude of the body
%   relative to the earth. It can be computed from the DCM. However, there
%   exist two solutions with reversed signs. In this function, the element
%   with the highest magnitude is defined to be positive.
% 
% Literature:
% [1]   Stevens, B. L. et al. (2016): Aircraft Control and Simulation.
%       Dynamics, Control Design, and Autonomous Systems. 3rd ed. Wiley.
% 
% Inputs:
%   M_bg            direction cosine matrix (with 3x3 elements) from NED
%                   frame (g) to body frame (b).
% 
% Outputs:
%   q_bg            quaternion vector with 4 elements and euclean norm of 1
% 
% See also: euler2Quat
% 

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

function q_bg  = dcm2Quat( M_bg ) %#codegen

% extract all elements of rotation matrix
m_11 = M_bg(1,1);
m_12 = M_bg(1,2);
m_13 = M_bg(1,3);
m_21 = M_bg(2,1);
m_22 = M_bg(2,2);
m_23 = M_bg(2,3);
m_31 = M_bg(3,1);
m_32 = M_bg(3,2);
m_33 = M_bg(3,3);

% compute signs for quaternions (multiple use)
sign_m23_minus = sign( m_23 - m_32 );
sign_m13_minus = sign( m_31 - m_13 );
sign_m12_minus = sign( m_12 - m_21 );
sign_m12_plus = sign( m_12 + m_21 );
sign_m23_plus = sign( m_23 + m_32 );
sign_m13_plus = sign( m_31 + m_13 );

% compute the magnitude of the quaternion elements according to [1, page
% 53]
q_0 = 1/2 * sqrtReal( 1 + m_11 + m_22 + m_33 );
q_1 = 1/2 * sqrtReal( 1 + m_11 - m_22 - m_33 );
q_2 = 1/2 * sqrtReal( 1 - m_11 + m_22 - m_33 );
q_3 = 1/2 * sqrtReal( 1 - m_11 - m_22 + m_33 );

% determine sign of the quaternion elements according to [1, page 53]
q_bg_unsigned = [ q_0; q_1; q_2; q_3 ];
[~, idx] = max(q_bg_unsigned);
idx = idx - 1;
switch idx
    case 0
        q_1 = sign_m23_minus * q_1;
        q_2 = sign_m13_minus * q_2;
        q_3 = sign_m12_minus * q_3;
    case 1
        q_0 = sign_m23_minus * q_0;
        q_2 = sign_m12_plus * q_2;
        q_3 = sign_m13_plus * q_3;
    case 2
        q_0 = sign_m13_minus * q_0;
        q_1 = sign_m12_plus * q_1;
        q_3 = sign_m23_plus * q_3;
    case 3
        q_0 = sign_m12_minus * q_0;
        q_1 = sign_m13_plus * q_1;
        q_2 = sign_m23_plus * q_2;
end

% build the quaternion vector
q_bg = [ q_0; q_1; q_2; q_3 ];

% assure that the euclidean norm equals 1
q_bg = quatNormalize( q_bg );

end