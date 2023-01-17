function xyz_cg = structureGetCg( structure, varargin )
% structureGetCg computes the center of gravity of a structure based on
%   the mass matrix and the node positions in the same frame as the node
%   positions.
% 
% Syntax:
%   xyz_cg = structureGetCg( structure )
%   xyz_cg = structureGetCg( structure, node_deflection )
%   xyz_cg = structureGetCg( structure, node_deflection, node_rotation )
% 
% Inputs:
%   structure           structure struct as specified by
%                       structureCreateFromNastran
%   node_deflection     (optional) position shift of the nodes due to 
%                       deformation (3xM array for M nodes)
%   node_rotation       (optional) rotation of the nodes due to deformation
%                       (3xM array)
% 
% Outputs:
%   xyz_cg              center of gravity position 3x1 vector in the same 
%                       frame as the nodes positions, in m
% 
% See also:
%   structureCreateFromNastran, structurePlot

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

node_rotation = zeros(size(structure.xyz));
if isempty(varargin)
    node_deflection = zeros(size(structure.xyz));
else
    node_deflection = varargin{1};
    if length(varargin) > 1
        node_rotation(:) = varargin{2};
    end
end

DOF         = 6;
n_nodes     = length( structure.M )/DOF;
mass_vector = structureGetNodeMass( structure, 1:n_nodes );
mass_total  = structureGetTotalMass( structure );
pos_node_cg = structure.xyz + node_deflection + structureGetNodeCg( ...
                structure, 1:n_nodes, node_rotation );
x_cg        = sum( mass_vector .* pos_node_cg(1,:) ) / mass_total;
y_cg        = sum( mass_vector .* pos_node_cg(2,:) ) / mass_total;
z_cg        = sum( mass_vector .* pos_node_cg(3,:) ) / mass_total;
xyz_cg      = [ x_cg; y_cg; z_cg ];

end