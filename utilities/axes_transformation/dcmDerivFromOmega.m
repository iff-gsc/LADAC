function M_ab_dt = dcmDerivFromOmega( M_ab, omega_ab_b ) %#codegen
% dcmDerivFromOmega computes the derivative of a rotation matrix (DCM) w.r.t. time [1].
% 
% Inputs:
%   M_ab            rotation matrix from b frame to a frame
%   omega_ab_b      angular velocity vector of frame b w.r.t. frame a
%                   represented in frame b, in rad/s
% 
% Outputs:
%   M_ab_dt         derivative of rotation matrix M_ab w.r.t. time
% 
% Literature:
%   [1] Zhao, S. (2016). Time derivative of rotation matrices: A tutorial.
%       arXiv preprint arXiv:1609.06088.
%       https://arxiv.org/pdf/1609.06088.pdf
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% skew matrix according to [1], eq. (1)
omega_b_skew = [ ...
    0, -omega_ab_b(3), omega_ab_b(2); ...
    omega_ab_b(3), 0, -omega_ab_b(1); ...
    -omega_ab_b(2), omega_ab_b(1), 0 ...
    ];

% time derivative of rotation matrix according to [1], eq. (3)
M_ab_dt = M_ab * omega_b_skew;

end