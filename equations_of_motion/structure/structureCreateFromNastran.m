function structure = structureCreateFromNastran( pch_filename, GRID_path, axis_reversed )
% structureCreateFromNastran initializes a structure struct from NASTRAN data.
%   The NASTRAN data is based on GRID files and a PCH file. The structure
%   struct contains all important data for structure dynamics.
% 
% Inputs:
%   pch_filename        name of PCH file containing the stiffness matrix
%                       and mass matrix (char)
%   GRID_path           path of the GRID files containing the node IDs and
%                       node positions (char)
%   axis_reversed       3x1 vector (x,y,z) with elements 
%                           1: axis not reversed
%                           -1: axis reversed
% 
% Outputs:
%   structure           structure struct with the following fields
%                       - K (stiffness nxn matrix)
%                       - M (mass nxn matrix)
%                       - xyz (node positions 3xn matrix)
% 
% See also:
%   structurePlot, structurePlotEigenmode, structureGetReduced

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

disp('structureCreateFromNastran: Starting to create structure from Nastran files...')

pch_contents    = nastranReadPch( pch_filename );
grid            = nastranReadGrid( GRID_path );
nodesPos        = nastranGetNodesPos( grid.xPos, grid.yPos, grid.zPos, grid.nodeIDs, pch_contents.nodesList );
sign_vector     = repmat( repmat(axis_reversed,2,1), length(nodesPos.x), 1 );
sign_matrix     = sign_vector * sign_vector';
structure.M     = sign_matrix .* pch_contents.M;
structure.K     = sign_matrix .* pch_contents.K;
structure.xyz   = axis_reversed.*[ nodesPos.x'; nodesPos.y'; nodesPos.z' ];

disp('structureCreateFromNastran: Finished!')

end