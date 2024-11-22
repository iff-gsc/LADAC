function I_node = structureGetNodeInertia( structure, idx )
% structureGetNodeInertia returns the inertia matrix for a specified
%   node in a structure struct. If multiple nodes are specified, a 3D inertia
%   matrix is returned.
% 
% Inputs:
%   structure       A structure struct as specified by
%                   structureCreateFromNastran
%   idx             Index of the node (1 to length(structure.xyz(1,:))),
%                   can be a scalar or a vector of integers
% 
% Outputs:
%   I_node          inertia matrix of the corresponding node(s) of the size
%                   3x3xn for length(idx) = n
% 

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

DOF = 6;
len_idx = length( idx );
I_node = zeros(3,3,len_idx);
for i = 1:len_idx
    idx_M = (4+(idx(i)-1)*DOF):(6+(idx(i)-1)*DOF);
    I_node(:,:,i) = structure.M(idx_M,idx_M);
end

end