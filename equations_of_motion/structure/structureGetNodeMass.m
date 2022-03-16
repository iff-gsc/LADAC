function mass_vector = structureGetNodeMass( structure, idx )
% structureGetNodeMass returns the mass for a specified node in a
%   structure struct. If multiple nodes are specified, a mass vector is
%   returned.
% 
% Inputs:
%   structure       A structure struct as specified by
%                   getStructureFromNastran
%   idx             Index of the node (1 to length(structure.xyz(1,:))),
%                   can be a scalar or a vector of integers
% 
% Outputs:
%   mass_vector     mass vector of the corresponding node(s) of the size
%                   1xn for length(idx) = n
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

DOF = 6;
idx_M = 1+(idx-1)*DOF;
mass_vector = diag( structure.M(idx_M,idx_M) )';

end