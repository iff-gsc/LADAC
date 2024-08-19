function M = structureGetNodesConnections( structure, varargin )
% structureGetNodesConnections returns a matrix of logicals that
%   indicates which nodes are connected with each other in a strucutre.
%   The connections are found by entries in the stiffness matrix.
%   At the moment this only works if all nodes have the same number of
%   degrees of freedom.
% 
% Inputs:
%   structure       structure struct as specified by
%                   getStructureFromNastran
%   tol             tolerance; a value lower than 1 which defines the
%                   minimum stiffness (relative to the maximum stiffness)
%                   that is considered as a connection.
% 
% Outputs:
%   M               interconnection matrix, where M(i,j) = true means that
%                   node i and node j are connected with each other
% 

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

tol = 0;
if ~isempty(varargin)
    tol = varargin{1};
end

K = structure.K;
K_max = max(max(K));
n_nodes = length( structure.xyz(1,:) );
DOF = 6;

M = false( n_nodes, n_nodes );

    for i = 1:n_nodes
        for j = 1:n_nodes
            % if any entry in the stiffness sub-matrix is not zero, the
            % nodes are connected with each other
            M(i,j) = any( abs( K(((i-1)*DOF+1):((i-1)*DOF+DOF),((j-1)*DOF+1):((j-1)*DOF+DOF)) ) > tol*K_max, 'all' );
            M(j,i) = any( abs( K(((j-1)*DOF+1):((j-1)*DOF+DOF),((i-1)*DOF+1):((i-1)*DOF+DOF)) ) > tol*K_max, 'all' );
        end
    end

end