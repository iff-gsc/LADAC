function [] = structurePlot( structure, varargin )
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

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

mass_color = 'm';
mass_size = 1;
line_width = 1;
structure_color = 'b';
node_size = 1;
tol = 0;
for i = 1:length(varargin)
    if strcmp(varargin{i},'MassColor')
        mass_color = varargin{i+1};
    elseif strcmp(varargin{i},'MassSize')
        mass_size = varargin{i+1};
    elseif strcmp(varargin{i},'LineWidth')
        line_width = varargin{i+1};
    elseif strcmp(varargin{i},'StructureColor')
        structure_color = varargin{i+1};
    elseif strcmp(varargin{i},'NodeSize')
        node_size = varargin{i+1};
    elseif strcmp(varargin{i},'Tolerance')
        tol = varargin{i+1};
    end
end

n_nodes = length( structure.xyz(1,:) );
M_int = structureGetNodesConnections( structure, tol );
xyz_cg = structure.xyz + structureGetNodeCg(structure,1:size(structure.xyz,2));

for i = 1:n_nodes
    % get indices of connected nodes
    [~,indices1] = find( M_int(i,:) == true );
    indices2 = indices1(indices1>=i);
    % plot connections of nodes
    if length(indices2)>1
        pairs = nchoosek(indices2, 2)';
        pairs(:,pairs(1,:)~=i)=[];
    else
        pairs = [];
    end
    plot3( structure.xyz(1,pairs), structure.xyz(2,pairs), structure.xyz(3,pairs), 'Color', structure_color, 'LineStyle', '-', 'LineWidth', line_width*1.5 )
    hold on
    plot3( structure.xyz(1,i), structure.xyz(2,i), structure.xyz(3,i), 'Color', structure_color, 'LineStyle', 'none', 'Marker', '.', 'MarkerSize', 13*node_size )
    % marker size should be proportional to third root of node mass
    mass = structureGetNodeMass( structure, i );
    MarkerSize = mass_size * max( 2*mass^(1/3), 0.1 );
    % plot nodes with marker size indicating the node mass
    scatter3( xyz_cg(1,i), xyz_cg(2,i), xyz_cg(3,i), MarkerSize.^2, 'MarkerFaceColor', mass_color, 'MarkerFaceAlpha', 0.5, 'MarkerEdgeColor', mass_color )
    % plot3( structure.xyz(1,i), structure.xyz(2,i), structure.xyz(3,i), 'mo', 'MarkerSize', MarkerSize, 'MarkerFaceColor', 'm' );
end

view(-150,25); % azimuth, elevation
set(gca, 'YDir','reverse')
set(gca, 'ZDir','reverse')

hold off
axis equal

end