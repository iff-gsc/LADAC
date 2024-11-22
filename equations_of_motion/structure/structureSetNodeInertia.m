function structure = structureSetNodeInertia( structure, I_node, idx )
% structureGetNodeInertia set the inertia matrix for a specified
%   node in a structure struct.
% 
% Inputs:
%   structure       A structure struct as specified by
%                   structureCreateFromNastran
%   I_node          inertia matrix of the corresponding node(s) of the size
%                   3x3xn for length(idx) = n
%   idx             Index of the node (1 to length(structure.xyz(1,:))),
%                   can be a scalar or a vector of integers
% 
% Outputs:
%   structure       A structure struct as specified by
%                   structureCreateFromNastran

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2024 Yannic Beyer
%   Copyright (C) 2024 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

DOF = 6;
len_idx = length( idx );
for i = 1:len_idx
    idx_M = (4+(idx(i)-1)*DOF):(6+(idx(i)-1)*DOF);
    structure.M(idx_M,idx_M) = I_node(:,:,i);
end

end