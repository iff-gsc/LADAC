function n_g = dcm2LeanVector( M_bg )
% dcm2LeanVector computes the reduced attitude vector in g frame (NED).
%   For more information see [1], section II.B.
% 
% Inputs:
%   M_bg                DCM from g frame to b frame
% 
% Outputs:
%   n_g                 unit vector that points into the -z_b direction
%                       represented in g frame (NED)
% 
% Literature:
%   [1] https://arxiv.org/pdf/2002.07837.pdf
% 
% See also:
%   quatReduced, euler2Dcm, quat2Dcm, dcm2Lean
% 

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% unit vector in body frame (pointing up)
n_b = [ 0; 0; -1 ];

n_g = M_bg' * n_b;

end