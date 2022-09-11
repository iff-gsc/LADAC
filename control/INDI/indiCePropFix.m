function [G10,G20] = indiCePropFix( cep, ceb, T_s ) %#codegen
% indiCePropFix computes the matrices G1 and G2 (see [1]) from 
%   the control effectiveness of motor-propeller control inputs.
%   G1 and G2 can be used for incremental inversion according to Eq. (19)
%   (neglecting G3).
%   Note that a 4th row is added to the control effectiveness matrix for
%   the desired vertical acceleration.
%   For scheduling of the control effectiveness w.r.t. the control input
%   itself, the function indiCePropVar should be used.
% 
% Inputs:
%   cep             propeller control effectiveness parameters struct, see
%                   indiCeProp_params_default
%   ceb             body control effectiveness parameters struct, see
%                   indiCeBody_params_default
%   T_s             controller sample time, s
% 
% Outputs:
%   G10             control effectiveness matrix according to [1], Eq. (9)
%   G20             control effectiveness matrix to compensate the control
%                   input derivative according to [2], Eq. (10)
% 
% See also:
%   indiCePropVar, indiCePropAffineCntrl
% 
% Literature:
%   [1] Smeur, E. J., Chu, Q., & de Croon, G. C. (2016). Adaptive
%       incremental nonlinear dynamic inversion for attitude control of
%       micro air vehicles. Journal of Guidance, Control, and Dynamics,
%       39(3), 450-461.

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

num_motors = length( cep.x );

nz = 1 - sqrtReal( cep.nx.^2 + cep.ny.^2 );
M = [ cep.nx; cep.ny; nz ];

P = [ cep.x; cep.y; cep.z ];

L = -cross( P, M );

I_b = [ ...
        ceb.ixx,    -ceb.ixy,	-ceb.ixz; ...
        -ceb.ixy,	ceb.iyy,	-ceb.iyz; ...
        -ceb.ixz,	-ceb.iyz,	ceb.izz ...
    ];

inv_I_b = inv(I_b);

% G1, see Eq. (9) in [1]
G10 = [ ...
        2 * inv_I_b * (cep.k*L + cep.d*M .* repmat(cep.a,3,1)); ...
        repmat( -2 * cep.k / ceb.m, 1, num_motors ) ...
    ];

% G2, see Eq. (10) in [1]
G20 = 1 / T_s * ...
    [ ...
        inv_I_b * cep.ip * M .* repmat(cep.a,3,1); ...
        zeros( 1, num_motors ) ...
    ];

end