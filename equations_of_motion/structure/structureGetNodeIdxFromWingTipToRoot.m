function idx = structureGetNodeIdxFromWingTipToRoot( structure, side )
% structureGetNodeIdxFromWingTipToRoot list of node indices from wing tip
% to wing root of a structure
%   Collect all node indices from wing tip to wing root in the idx array.
%   Therefore we start at the wing tip and then repeat the following:
%   Find all connected nodes by looking into the stiffness matrix and go to
%   the connected node that is the closest to the wing tip. Do not go to 
%   nodes that are already in the idx array. If a deadlock is reached, we can
%   go backwards until we find new nodes (like this, duplicate numbers appear
%   in the idx array). The search is finished when we reach the aircraft
%   center (0.01*y_tip).
% 
% Syntax:
%   idx = structureGetNodeIdxFromWingTipToRoot( structure, side )
% 
% Inputs:
%   structure               structure struct (see
%                           structureCreateFromNastran)    
%   side                    desired wing side: 'left' or 'right'
% 
% Outputs:
%   idx                     array of indices, where the first element is
%                           the wing tip node index and the last element is
%                           the wing root node index (Nx1 array for N
%                           nodes)
% 
% See also:
%   structureGetBendingMomentTrafoAt, structureCreateFromNastran 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2021-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

switch side
    case 'left'
        [~,idx_wing_tip_node] = min(structure.xyz(2,:));
    case 'right'
        [~,idx_wing_tip_node] = max(structure.xyz(2,:));
end
idx = idx_wing_tip_node;
num_nodes = size( structure.xyz, 2 );
K_3d = reshape( structure.K, 6, num_nodes, 6, num_nodes );
steps_back = 0;
while true
    stiffness_col = reshape( squeeze( K_3d(:,idx(end),:,:)), 6*6, num_nodes )';
    K_max = max(abs(stiffness_col));
    [idx_connected_nodes,~] = find( abs(stiffness_col)>repmat(1e-3*K_max,size(stiffness_col,1),1) );
    idx_connected_nodes_new = removeEntries(idx_connected_nodes,idx);
    if ~isempty(idx_connected_nodes_new)
        steps_back = 0;
        switch side
            case 'left'
                [~,idx_next_node_sub] = min(structure.xyz(2,idx_connected_nodes_new));
            case 'right'
                [~,idx_next_node_sub] = max(structure.xyz(2,idx_connected_nodes_new));
        end
        idx_next_node = idx_connected_nodes_new(idx_next_node_sub);
        idx = [idx;idx_next_node];
    else
        % if we are in a deadlock, we go backwards until we find new nodes
        % again
        steps_back = steps_back + 1;
        idx = [idx;idx(end-steps_back)];
        steps_back = steps_back + 1;
    end
    switch side
        case 'left'
            is_root_reached = structure.xyz(2,idx(end)) > 0.01 * structure.xyz(2,idx_wing_tip_node);
        case 'right'
            is_root_reached = structure.xyz(2,idx(end)) < 0.01 * structure.xyz(2,idx_wing_tip_node);
    end
    if is_root_reached
        idx(end) = [];
        break;
    end
end


function idx = removeEntries( idx, idx_remove )
    for i = 1:length(idx_remove)
        idx(idx==idx_remove(i)) = [];
    end
    idx = unique(idx);
end

end
