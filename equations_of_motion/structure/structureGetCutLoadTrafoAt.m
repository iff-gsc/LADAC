function [md2cl,span_load_station] = structureGetCutLoadTrafoAt( ...
    structure_red, structure, eta, varargin )
% structureGetCutLoadTrafoAt compute transformation matrix that maps
% modal dispacement vector to cut loads at specified locations
% 
% Syntax:
%   md2cl = structureGetCutLoadTrafoAt( structure_red, structure, eta )
%   md2cl = structureGetCutLoadTrafoAt( structure_red, structure, eta, load_type )
%   md2cl = structureGetCutLoadTrafoAt( structure_red, structure, eta, reverse_side )
% 	md2cl = structureGetCutLoadTrafoAt( structure_red, structure, eta, load_type, reverse_side )
%   md2cl = structureGetCutLoadTrafoAt( structure_red, structure, eta, reverse_side, load_type )
% 
% Inputs:
%   structure_red           reduced-order structure struct (see
%                           structureGetReduced)
%   structure               structure struct (see
%                           structureCreateFromNastran)
%   eta                     dimensionless spanwise locations where the
%                           bending moment should be evaluated (1xN array)
%   load_type               (optional) desired type of cut load (string):
%                               'Fx' (shear force in x direction)
%                               'Fy' (shear force in y direction)
%                               'Fz' (shear force in z direction)
%                               'Mx' (bending moment about x axis)
%                               'My' (torsional moment about y axis)
%                               'Mz' (bending moment about z axis)
%                               'all' (all of the above loads - default)
%                           Note that the moments reference point is the
%                           specified eta (in y direction) and on the line
%                           of structure nodes (in x and z direction).
%   reverse_side            (optional) In symmetric flight the cut loads
%                           'Fy', 'Mx' and 'Mz' have different signs on the
%                           left wing and on the right wing. This input can
%                           be used to specify whether the sign of the
%                           named cut loads should be reversed:
%                               'reverse_left' (reverse for left wing)
%                               'reverse_right' (reverse for right wing -
%                                               default)
%                               'reverse_none' (no sign reversal)
% 
% Outputs:
%   md2cl                   transformations matrix that maps the modal
%                           displacement vector to the cut load at
%                           each eta (NxM array, where M is the number of
%                           modal displacements of structure_red and N is
%                           the length of eta (in case of load_type 'all',
%                           N is 6 times the length of eta)); it is the
%                           matrix product of [1], eq. (11)
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

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

load_type = 'all';
reverse_side = 'reverse_right';
for i = 1:length(varargin)
    if ischar(varargin{i})
        if isequal(varargin{i},'all')
            load_type = 'all';
        elseif isequal(varargin{i},'Fx')
            load_type = 'Fx';
        elseif isequal(varargin{i},'Fy')
            load_type = 'Fy';
        elseif isequal(varargin{i},'Fz')
            load_type = 'Fz';
        elseif isequal(varargin{i},'Mx')
            load_type = 'Mx';
        elseif isequal(varargin{i},'My')
            load_type = 'My';
        elseif isequal(varargin{i},'Mz')
            load_type = 'Mz';
        elseif isequal(varargin{i},'reverse_none')
            reverse_side = 'reverse_none';
        elseif isequal(varargin{i},'reverse_left')
            reverse_side = 'reverse_left';
        elseif isequal(varargin{i},'reverse_right')
            reverse_side = 'reverse_right';
        end
    end
end

switch load_type
    case 'all'
        type_idx = 1:6;
    case 'Fx'
        type_idx = 1;
    case 'Fy'
        type_idx = 2;
    case 'Fz'
        type_idx = 3;
    case 'Mx'
        type_idx = 4;
    case 'My'
        type_idx = 5;
    case 'Mz'
        type_idx = 6;
end

Phi_Lg = zeros( length(eta)*length(type_idx), size( structure.K,1 ) );
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
    
    [y_unique,idx_y_unique] = unique( wing_nodes_pos_tip_to_root(2,:) );
    wing_nodes_pos_unique = wing_nodes_pos_tip_to_root(:,idx_y_unique);
    span_pos = abs(eta(i)) * wing_nodes_pos_tip_to_root(2,1);
    xyz_lever = structure.xyz(:,idx_use) ...
        - interp1( y_unique', wing_nodes_pos_unique', span_pos, ...
                'linear', 'extrap' )';
    
    % reverse sign on left or right wing
    eye_force_sign      = eye(3);
    eye_moment_sign     = eye(3);
    eye_force_sign_rev  = diag([1,-1,1]);
    eye_moment_sign_rev = diag([-1,1,-1]);
    switch reverse_side
        case 'reverse_left'
            if eta(i) < 0
                eye_force_sign  = eye_force_sign_rev;
                eye_moment_sign = eye_moment_sign_rev;
            end
        case 'reverse_right'
            if eta(i) > 0
                eye_force_sign  = eye_force_sign_rev;
                eye_moment_sign = eye_moment_sign_rev;
            end
    end
    
    for j = 1:size(xyz_lever,2)
        Phi_tmp = [ eye_force_sign, zeros(3,3); eye_moment_sign * vec2Skew(xyz_lever(:,j)), eye_moment_sign ];
        idx_1 = (i-1)*length(type_idx) + (1:length(type_idx));
        idx_2 = (idx_use(j)-1)*6 + (1:6);
        Phi_Lg( idx_1, idx_2 ) = Phi_tmp(type_idx,:);
    end
    
    span_load_station(i) = eta(i) * wing_nodes_pos_tip_to_root(2,1);

end

% matrix product of [1], eq. (11)
md2cl = Phi_Lg * structure.K * structure_red.modal.T;

end

function mat = vec2Skew( vec )
    mat = [ 0,-vec(3),vec(2); vec(3),0,-vec(1); -vec(2),vec(1),0 ];
end