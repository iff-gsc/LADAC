function [] = structurePlot( structure )
% structurePlot plots the structure based on the nodes positions, the
%   stiffness matrix and the mass matrix. All connected nodes are
%   visualized by lines, the node is visualized by a dot and a circle where
%   the circle size is correlated with the node mass.
% 
% Syntax:
%   structurePlot( structure )
% 
% Inputs:
%   structure           structure struct as specified by
%                       structureCreateFromNastran
% 
% Outputs:
%   -
% 
% See also:
%   structureCreateFromNastran, structurePlotEigenmode

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

n_nodes = length( structure.xyz(1,:) );
M_int = structureGetNodesConnections( structure );

for i = 1:n_nodes
    % get indices of connected nodes
    [~,indices1] = find( M_int(i,:) == true );
    % plot connections of nodes
    pairs = nchoosek(indices1, 2)';
    plot3( structure.xyz(1,pairs), structure.xyz(2,pairs), structure.xyz(3,pairs), 'b.-', 'LineWidth', 1.5, 'MarkerSize', 13 )
    hold on
    % marker size should be proportional to third root of node mass
    mass = structureGetNodeMass( structure, i );
    MarkerSize = max( 2*mass^(1/3), 0.1 );
    % plot nodes with marker size indicating the node mass
    scatter3( structure.xyz(1,i), structure.xyz(2,i), structure.xyz(3,i), MarkerSize.^2, 'MarkerFaceColor', 'm', 'MarkerFaceAlpha', 0.5, 'MarkerEdgeColor', 'm' )
    % plot3( structure.xyz(1,i), structure.xyz(2,i), structure.xyz(3,i), 'mo', 'MarkerSize', MarkerSize, 'MarkerFaceColor', 'm' );
end

view(-150,25); % azimuth, elevation
set(gca, 'YDir','reverse')
set(gca, 'ZDir','reverse')

hold off
axis equal

end