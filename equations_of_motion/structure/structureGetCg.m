function xyz_cg = structureGetCg( structure )
% structureGetCg computes the center of gravity of a structure based on
%   the mass matrix and the node positions in the same frame as the node
%   positions.
% 
% Inputs:
%   structure           structure struct as specified by
%                       getStructureFromNastran
% 
% Outputs:
%   xyz_cg              center of gravity position 3x1 vector in the same 
%                       frame as the nodes positions
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

DOF         = 6;
n_nodes     = length( structure.M )/DOF;
mass_vector = structureGetNodeMass( structure, [1:n_nodes] );
mass_total  = structureGetTotalMass( structure );
x_cg        = sum( mass_vector .* structure.xyz(1,:) ) / mass_total;
y_cg        = sum( mass_vector .* structure.xyz(2,:) ) / mass_total;
z_cg        = sum( mass_vector .* structure.xyz(3,:) ) / mass_total;
xyz_cg      = [ x_cg; y_cg; z_cg ];

end