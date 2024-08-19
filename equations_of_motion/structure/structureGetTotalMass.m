function mass_total = structureGetTotalMass( structure )
% structureGetTotalMass computes the mass of the complete structure
%   struct based on the mass matrix.
% 
% Inputs:
%   structure           structure struct as specified by
%                       getStructureFromNastran
% 
% Outputs:
%   mass_total          mass (scalar), sum of the mass of all nodes
% 

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

DOF         = 6;
n_nodes     = length( structure.M )/DOF;
mass_vector = structureGetNodeMass( structure, [1:n_nodes] );
mass_total = sum( mass_vector );

end