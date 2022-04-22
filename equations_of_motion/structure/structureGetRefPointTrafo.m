function T_ref_s = structureGetRefPointTrafo(structure,xyz_ref)

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

% find closest node to reference points
[~, idx] = sort( vecnorm( xyz_diff, 2, 1 ) );

% in three directions find the closest nodes but avoid duplicate
% coordinates
idx_x = [idx(1),0];
idx_y = [idx(1),0];
idx_z = [idx(1),0];
ix = 2;
iy = 2;
iz = 2;
while true
    if idx_x(2) ~= 0 && idx_y(2) ~= 0 && idx_z(2) ~= 0
        break
    else
        if idx_x(2) == 0
            ix = ix+1;
            if structure.xyz(1,idx(ix)) ~= structure.xyz(1,idx_x(1))
                idx_x(2) = idx(ix);
            end
        end
        if idx_y(2) == 0
            iy = iy+1;
            if structure.xyz(2,idx(iy)) ~= structure.xyz(2,idx_y(1))
                idx_y(2) = idx(iy);
            end
        end
        if idx_z(2) == 0
            iz = iz+1;
            if structure.xyz(3,idx(iz)) ~= structure.xyz(3,idx_z(1))
                idx_z(2) = idx(iz);
            end
        end
    end
end

idx_T_x = node2StateIdx( idx_x, 1 );
idx_T_y = node2StateIdx( idx_y, 2 );
idx_T_z = node2StateIdx( idx_z, 3 );

% linear interpolation for angular acceleration
T_ref_s_angularAccel(1,idx_T_x) = getLinInterpMatrix( structure.xyz(1,idx_x), xyz_ref(1) );
T_ref_s_angularAccel(2,idx_T_y) = getLinInterpMatrix( structure.xyz(2,idx_y), xyz_ref(2) );
T_ref_s_angularAccel(3,idx_T_z) = getLinInterpMatrix( structure.xyz(3,idx_z), xyz_ref(3) );
% linear interpolation for linear acceleration (This part could also
% consider linear acceleartion due to angular acceleration times lever arm.
% However, this effect is neglected.)
T_ref_s_linearAccel(1,idx_T_x) = getLinInterpMatrix( structure.xyz(1,idx_x), xyz_ref(1) );
T_ref_s_linearAccel(2,idx_T_y) = getLinInterpMatrix( structure.xyz(2,idx_y), xyz_ref(2) );
T_ref_s_linearAccel(3,idx_T_z) = getLinInterpMatrix( structure.xyz(3,idx_z), xyz_ref(3) );

% compute combined transformation matrix (modal -> structure nodes ->
% reference point)
T_ref_s = [ T_ref_s_linearAccel; T_ref_s_angularAccel ] * structure.modal.T;

end

function M = getLinInterpMatrix(x,xq)
% to do: function copied from wingSetAeroelasticity

len_x = length(x);
len_xq = length(xq);

M = zeros(len_xq,len_x);

for i = 1:len_xq
    xs_left = find(x<xq(i));
    xs_right = find(x>=xq(i));
    if isempty(xs_left)
        % allow extrapolation to left
        x_left = 1;
        x_right = 2;
    elseif isempty(xs_right)
        % allow extrapolation to right
        x_left = len_x-1;
        x_right = len_x;
    else
        % interpolation
        x_left = xs_left(end);
        x_right = xs_right(1);
    end
    factor_linear_interp = 1/(x(x_right)-x(x_left)) ...
        * (xq(i)-x(x_left));
    M(i,x_left) = 1 - factor_linear_interp;
    M(i,x_right) = factor_linear_interp;
end

end

function state_idx = node2StateIdx( node_idx, node_state_idx )
    state_idx = (node_idx-1)*6 + node_state_idx;
end