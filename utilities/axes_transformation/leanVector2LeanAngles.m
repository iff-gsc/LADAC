function [phi,delta] = leanVector2LeanAngles(n_g)
% leanVector2LeanAngles converts the reduced attitude represented as unit
%   vector into the reduced attitude represented as angles.
%   For more information about the reduced attitude vector see [1], section
%   II.B.
% 
% Inputs:
%   n_g                 unit vector that points into the -z_b direction
%                       represented in g frame (NED)
% 
% Outputs:
%   phi                 tilt angle (see dcm2Lean), rad
%   delta               direction of the tilt angle (see dcm2Lean), rad
% 
% Literature:
%   [1] Sun, S., Wang, X., Chu, Q., & de Visser, C. (2020). Incremental
%       nonlinear fault-tolerant control of a quadrotor with complete loss
%       of two opposing rotors. IEEE Transactions on Robotics, 37(1),
%       116-130.
% 
% See also:
%   quatReduced, euler2Dcm, quat2Dcm, leanAngles2LeanVector

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

phi = acosReal(-n_g(3));
delta = atan2(n_g(2),n_g(1));
end