function I_node = structureGetNodeInertiaRef( structure, idx, pos_ref ) %#codegen
% structureGetNodeInertiaRef computes the moment of inertia tensor for
%   specified nodes of the structure w.r.t. a reference point.
% 
% Inputs:
%   structure           structure struct as specified by
%                       getStructureFromNastran
%   idx                 Index of the node (1 to length(structure.xyz(1,:))),
%                       can be a scalar or a vector of integers
%   pos_ref             reference position (3x1) vector, in m
% 
% Outputs:
%   I                   moment of inertia (3x3) tensor with respect to the
%                       center of mass in the same frame as the nodes
%                       positions, in kg.m^2
% 

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

len_idx = length( idx );
I_node = zeros(3,3,len_idx);
for i = 1:len_idx
    
    % compute inertia tensor according to https://de.wikipedia.org/wiki/Steinerscher_Satz
    xyz_cg_dist = structure.xyz(:,i) - pos_ref;
    a = [ 0, -xyz_cg_dist(3), xyz_cg_dist(2);
            xyz_cg_dist(3), 0 , -xyz_cg_dist(1)
            -xyz_cg_dist(2), xyz_cg_dist(1), 0 ];
    I_node(:,:,i) = structureGetNodeInertia( structure, i ) + ...
        structureGetNodeMass( structure, i ) * (a'*a);

end