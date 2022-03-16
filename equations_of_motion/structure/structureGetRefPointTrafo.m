function [T_ref_s,r_node_ref,idx] = structureGetRefPointTrafo(structure,xyz_ref)

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% number of nodes
numNodes = size( structure.xyz, 2 );

% init sub transformation matrices
T_ref_s_angularAccel = zeros( 3, numNodes*6 );
T_ref_s_linearAccel = zeros( 3, numNodes*6 );

% distance vectors from nodes to reference point
xyz_diff = xyz_ref - structure.xyz;

% find closest node to reference point
[~, idx] = min( vecnorm( xyz_diff, 2, 1 ) );

% vector from closest node to reference point
r_node_ref = xyz_diff(:,idx);

% state indices from node index
idx_states = (idx-1)*6+1:idx*6;

% angular acceleration is equal for all points on rigid body
T_ref_s_angularAccel(:,idx_states(4:6)) = diag(ones(3,1));
% for linear acceleration of point on rigid body, both the linear
% acceleration and the angular acceration must be considered (cross product)
T_ref_s_linearAccel(:,idx_states(1:3)) = diag(ones(3,1));
% T_ref_s_linearAccel(:,idx_states(4:6)) = [ ...
%     0, r_node_ref(3), -r_node_ref(2); ...
%     -r_node_ref(3),  0, r_node_ref(1); ...
%     r_node_ref(2), -r_node_ref(1), 0 ...
%     ];

% compute combined transformation matrix (modal -> structure nodes ->
% reference point)
T_ref_s = [ T_ref_s_linearAccel; T_ref_s_angularAccel ] * structure.modal.T;

end