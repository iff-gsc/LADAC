function [ structure_red, omega_red, V, D ] = structureGetReduced( structure, N )
% structureGetReduced computes a structure struct from a structure struct
% with reduced order.
%   The mode displacement method is used to reduce the order of the
%   structural model according to [1].
% 
% Inputs:
%   structure       structure struct as defined in
%                   structureCreateFromNastran
%   N               order of the reduced model (must be greater equal 1 and
%                   less equal than the order of structure)
% 
% Outputs:
%   structure_red   structure struct with the reduced order model
%   omega_red       vector of eigenfrequencies of the reduced order model
%                   ascending order, in rad/s
%   V               matrix whose columns are the eigenvectors of structure
%   D               diagonal matrix whose diagonal elements are the
%                   corresponding eigenvalues, in Hz^2
% 
% Literature:
%   [1] Besselink, B., Tabak, U., Lutowska, A., van de Wouw, N., Nijmeijer,
%       H., Rixen, D. J., ... & Schilders, W. H. A. (2013). A comparison of
%       model reduction techniques from structural dynamics, numerical
%       mathemetrics and systems and control. Journal of Sound and
%       Vibration, 332(19), 4403-4422.
% 

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% get eigenvalues of eq. (3) in [1], eigenvalues (omega^2) are in (rad/s)^2
[V,D] = eig( structure.K, structure.M );

% convert diagonal matrix of eigenvalues into a vector of eigenfrequencies
% in rad/s
[omega,idx_omega] = sort(sqrt(diag(abs(D))));

% number of degrees of freedom for every node
DOF = 6;

% get vector of reduced eigenfrequencies
omega_red = omega((DOF+1):(DOF+N));

% compute transformation matrix according to [1], eq. (8)
T = real(V(:,idx_omega(1:(DOF+N))));

% compute reduced stiffness matrix according to [1], eq. (11)
Kr = T' * structure.K * T;
% compute reduced mass matrix according to [1], eq. (10)
Mr = T' * structure.M * T;

% set structure struct
structure_red.K = Kr;
structure_red.M = Mr;
structure_red.d = zeros(DOF+N,1);
structure_red.M_inv = pinv( structure_red.M );
structure_red.xyz = structure.xyz;
structure_red.modal.T = T;
structure_red.modal.omega_red = omega_red;

end