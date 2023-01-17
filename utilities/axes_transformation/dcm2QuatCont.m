% dcm2Quat computes the quaternion (q_bg) from the direction cosine matrix
% (DCM) so that there are no discontinuities.
%   The quaternion is used to represent the attitude of the body
%   relative to the earth. It can be computed from the DCM. However, there
%   exist two solutions with reversed signs. This function avoids steps
%   which can be important for controllers.
% 
% Literature:
%   [1] Wu, J. (2018). Optimal Continuous Unit Quaternions from Rotation
%       Matrices. Journal of Guidance, Control and Dynamics.
% 
% Inputs:
%   M_bg            direction cosine matrix (with 3x3 elements) from NED
%                   frame (g) to body frame (b).
% 
% Outputs:
%   q_bg            quaternion vector with 4 elements and euclean norm of 1
% 
% See also: dcm2Quat
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

function q_bg  = dcm2QuatCont( M_bg, last_q ) %#codegen

% get quaternions (with discontinuities)
q_bg = dcm2Quat( M_bg );

% remove discontinuities
q_bg = sign(q_bg' * last_q) * q_bg;

end