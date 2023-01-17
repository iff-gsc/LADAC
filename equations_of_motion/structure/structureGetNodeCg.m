function cg_node = structureGetNodeCg( structure, idx, varargin )
% structureGetNodeCg returns the center of gravity (cg) of a node.
% 
% Inputs:
%   structure       A structure struct as specified by
%                   getStructureFromNastran
%   idx             Index of the node (1 to length(structure.xyz(1,:))),
%                   can be a scalar or a vector of integers
% 
% Outputs:
%   cg_node         center of gravity position of the corresponding node(s)
%                   in structure frame w.r.t. node position of the size 3xn
%                   for length(idx) = n

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% to do: consider rotation
if isempty(varargin)
    node_rotation = zeros(size(structure.xyz));
else
    node_rotation = varargin{1};
end

DOF = 6;
len_idx = length( idx );
cg_node = zeros(3,len_idx);
m_node = structureGetNodeMass(structure,idx);
for i = 1:len_idx
    idx_M = (4+(idx(i)-1)*DOF):(6+(idx(i)-1)*DOF);
    coupling_mat = structure.M(idx_M,idx_M-3);
    coupling_x = coupling_mat(2,3);
    coupling_y = coupling_mat(1,3);
    coupling_z = coupling_mat(2,1);
    cg_node(1,i) = -divideFinite( coupling_x, m_node(i) );
    cg_node(2,i) = -divideFinite( coupling_y, m_node(i) );
    cg_node(3,i) = divideFinite( coupling_z, m_node(i) );
end

end