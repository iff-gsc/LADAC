function structure = structureSetNodeCg( structure, idx, cg_node )
% structureSetNodeCg adjusts the center of gravity of nodes within a
% structural dynamics modal
%   If the center of gravity does not coincident with the node position,
%   there will be coupling terms between translational accelerations and
%   rotory accelerations within the mass matrix of a linear structural
%   dynamics model. By specifying the center of gravity position, these
%   coupling terms are set by this function.
% 
% Syntax:
%   structure = structureSetNodeCg( structure, idx, cg_node )
% 
% Inputs:
%   structure       structure struct as specified by
%                   structureCreateFromNastran
%   idx             Index of the node(s) (1 to length(structure.xyz(1,:))),
%                   can be a scalar or a vector of integers
%   cg_node         Center of gravity position of the corresponding node(s)
%                   in structure frame w.r.t. node position of the size 3xn
%                   for length(idx) = n
% 
% Outputs:
%   structure       structure struct as specified by
%                   structureCreateFromNastran
% 
% See also:
%   structureCreateFromNastran, structureGetNodeCg

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2024 Yannic Beyer
%   Copyright (C) 2024 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

DOF = 6;
len_idx = length( idx );
m_node = structureGetNodeMass(structure,idx);
for i = 1:len_idx
    idx_M = (4+(idx(i)-1)*DOF):(6+(idx(i)-1)*DOF);
    
    coupling_x = -m_node(i) * cg_node(1,i);
    coupling_y = -m_node(i) * cg_node(2,i);
    coupling_z = -m_node(i) * cg_node(3,i);
    coupling_mat = zeros(3,3);
    coupling_mat(2,3) = coupling_x;
    coupling_mat(3,2) = -coupling_x;
    coupling_mat(1,3) = coupling_y;
    coupling_mat(3,1) = coupling_y;
    coupling_mat(2,1) = coupling_z;
    coupling_mat(1,2) = coupling_z;
    
    structure.M(idx_M,idx_M-3) = coupling_mat;
    structure.M(idx_M-3,idx_M) = -coupling_mat;
    
end

end