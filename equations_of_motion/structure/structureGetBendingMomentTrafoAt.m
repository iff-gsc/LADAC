function [md2bm, span_load_station] = structureGetBendingMomentTrafoAt( structure_red, structure, eta )
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
%   span_load_station       y coordinates in aircraft frame of the load
%                           stations defined by eta (1xN array)
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

Phi_Lg = zeros( length(eta), size( structure.K,1 ) );
span_load_station = zeros(1,length(eta));

% node indices from wing tip to wing root
idx_left    = structureGetNodeIdxFromWingTipToRoot( structure, 'left' );
idx_right   = structureGetNodeIdxFromWingTipToRoot( structure, 'right' );

% for each eta one row in the transformation matrix is computed
for i = 1:length(eta)
    % We check from wing root when we reach the specified eta.
    % All remaining node indices will be considered to compute the bending
    % moment. However, we have to use unique since idx_side could contain
    % duplicate numbers.
    if eta(i) < 0
        idx_side = idx_left;
    else
        idx_side = idx_right;
    end
    wing_nodes_pos_tip_to_root = structure.xyz(:,idx_side);
    is_outside_flip = abs(flip(wing_nodes_pos_tip_to_root(2,:))) > abs(eta(i) * wing_nodes_pos_tip_to_root(2,1));
    idx_outside = idx_side(flip(is_outside_flip));
    idx_use = unique(idx_outside);
    
    % consider y lever arm and z force to compute bending moment
    Phi_Lg( i, (idx_use-1)*6+3 ) = -abs( structure.xyz(2,idx_use) - abs(eta(i)) * wing_nodes_pos_tip_to_root(2,1) );
%     Phi_Lg( i, (idx_use-1)*6+2 ) = -abs( structure.xyz(2,idx_use) - abs(eta(i)) * wing_nodes_pos_tip_to_root(3,1) );
%     Phi_Lg( i, (idx_use-1)*6+4 ) = 1;

    span_load_station(i) = eta(i) * wing_nodes_pos_tip_to_root(2,1);
end


% matrix product of [1], eq. (11)
md2bm = Phi_Lg * structure.K * structure_red.modal.T;

end
