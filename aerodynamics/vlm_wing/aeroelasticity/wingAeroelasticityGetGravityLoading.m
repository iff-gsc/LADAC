function R_Gb_i = wingAeroelasticityGetGravityLoading( wing, structure, M_bg )
% structureGetBendingMomentTrafoAt compute transformation matrix that maps
% modal dispacement vector to bending moment at specified locations
% 
% Inputs:
%   structure_red           reduced-order structure struct (see
%                           structureGetReduced)
%   structure               structure struct (see
%                           structureCreateFromNastran)
%   eta                     dimensionless spanwise locations where the
%                           bending moment should be evaluated (1xN array)
% 
% Outputs:
%   md2bm                   transformations matrix that maps the modal
%                           displacement vector to the bending moment at
%                           each eta (NxM array, where M is the number of
%                           modal displacements of structure_red); it is
%                           the matrix product of [1], eq. (11)
% 
% See also:
%   structureGetReduced, structureCreateFromNastran
% 
% Literature:
%   [1] Moulin, B., & Karpel, M. (2007). Gust loads alleviation using 
%       special control surfaces. Journal of Aircraft, 44(1), 17-25.
%       https://arc.aiaa.org/doi/pdf/10.2514/1.19876
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************


m_nodes = structureGetNodeMass(structure,1:size(structure.xyz,2));

g_g = [0;0;9.81];

g_b = M_bg * g_g;

% force of gravity
p_gravity_b = [ ...
    g_b * m_nodes; ...
    zeros(3,size(m_nodes,2)) ...
    ];

p = reshape( p_gravity_b, [], 1 );

[aeroelasticity,T_cs_f] = wingSetAeroelasticity( wing.geometry, structure, 0 );

R_Gb_i = zeros( 3, wing.n_panel );

% scaling = diff([0,structure_eta_mid,1])' * 1./diff(wing_eta);

R_Gb_i(1,:) = T_cs_f(1:4:end,1:6:end) * p(1:6:end);
R_Gb_i(2,:) = T_cs_f(2:4:end,2:6:end) * p(2:6:end);
R_Gb_i(3,:) = T_cs_f(3:4:end,3:6:end) * p(3:6:end);

end