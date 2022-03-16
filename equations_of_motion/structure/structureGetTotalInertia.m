function I = structureGetTotalInertia( structure ) %#codegen
% structureGetTotalInertia computes the moment of inertia tensor of the
%   complete structure based on the mass matrix and the node positions.
% 
% Inputs:
%   structure           structure struct as specified by
%                       getStructureFromNastran
% 
% Outputs:
%   I                   moment of inertia (3x3) tensor with respect to the
%                       center of mass in the same frame as the nodes
%                       positions, in kg.m^2
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

xyz_cg      = structureGetCg( structure );
n_nodes     = length( structure.xyz(1,:) );

I = zeros( 3, 3 );
for i = 1:n_nodes
    
    % compute inertia tensor according to https://de.wikipedia.org/wiki/Steinerscher_Satz
    xyz_cg_dist = structure.xyz(:,i) - xyz_cg;
    a = [ 0, -xyz_cg_dist(3), xyz_cg_dist(2);
            xyz_cg_dist(3), 0 , -xyz_cg_dist(1)
            -xyz_cg_dist(2), xyz_cg_dist(1), 0 ];
    I = I + ...
        structureGetNodeInertia( structure, i ) + ...
        structureGetNodeMass( structure, i ) * (a'*a);

end