function fuselage = fuselageSetAeroelasticity( fuselage, structure, is_modal )
% fuselageSetAeroelasticity set aeroelasticity struct in fuselage struct
%   For aeroelastic simulations the structure node positions must be
%   converted into aerodynamic node positions and vice versa. Therefore,
%   transformation matrices are computed in this function that map from
%   structure points to aerodynamic points and vice versa.
%   First, the nodes are assigned to each other (assumes that structure
%   geometry and aerodynamic geometry match). Then, coefficients for linear
%   interpolations are computed and stored inside the matrices.
%   Note that the structure state is usually expressed in modal
%   coordinates to reduce the order of states. That is why, an additional
%   transformation is performed.
% 
% Syntax:
%   fuselage = fuselageSetAeroelasticity( fuselage, structure, is_modal )
% 
% Inputs:
%   fuselage        fuselage struct (see fuselageInit)
%   structure     	a structure struct (see structureCreateFromNastran)
%   is_modal        logical if the structure is based on modal coordinates
%                   (should be true)
% 
% Outputs:
%   fuselage        fuselage struct (see fuselageInit)
% 
% See also:
%   fuselageInit, structureCreateFromNastran, fuselageAssignToStructure
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% compute relative positions to fuselage center line (xi)
segment_length = vecnorm( diff( fuselage.geometry.border_pos, 1, 2 ), 2, 1 );
cntrl_shift = vecnorm( diff( fuselage.geometry.cntrl_pos - fuselage.geometry.border_pos(:,1:end-1), 2, 1 ), 2, 1 );
total_line_length = sum( segment_length );
fuse_xi = cumsum( [0, segment_length ] ) / total_line_length;
fuse_xi_c = ( cumsum( [ 0, segment_length(1:end-1) ] ) + cntrl_shift ) / total_line_length;

assignment_vector = fuselageAssignToStructure(fuselage.geometry,structure);

fuse_idx = assignment_vector(assignment_vector~=0);
nodes_pos = structure.xyz(:,assignment_vector~=0);
num_nodes = sum(assignment_vector~=0);
num_nodes_total = length(assignment_vector);
num_border = length(fuse_xi);
num_cntrl_pt = length(fuse_xi_c);

[fuse_idx_sort,idx_sort] = sort(fuse_idx);
nodes_pos_sort = nodes_pos(:,idx_sort);

structure_xi = zeros(1,num_nodes);

for i = 1:num_nodes
    xi_front = fuse_xi(fuse_idx_sort(i));
    Delta_pos = norm(nodes_pos_sort(1,i)-fuselage.geometry.border_pos(1,fuse_idx_sort(i)),2);
    Delta_pos_fuse = segment_length(fuse_idx_sort(i));
    Delta_xi = diff(fuse_xi(fuse_idx_sort(i):fuse_idx_sort(i)+1));
    structure_xi(i) = xi_front + Delta_pos/Delta_pos_fuse * Delta_xi;
end
% re-order (necessary for nodes at same panels)
[structure_xi,idx_sort_2] = sort(structure_xi);
fuse_idx_sort = fuse_idx_sort(idx_sort_2);
idx_sort = idx_sort(idx_sort_2);
nodes_pos_sort = nodes_pos_sort(:,idx_sort_2);
structure_xi_mid = structure_xi(1:end-1) + diff(structure_xi)/2;

% build local linear interpolation matrix (also extrapolation)
T_b = getLinInterpMatrix(structure_xi,fuse_xi);
T_c = getLinInterpMatrix(structure_xi,fuse_xi_c);
M_cs = getLinInterpMatrix(fuse_xi_c,structure_xi);
% force distribution must be equal but force per node depends on number of
% nodes (scaling depending on panel length)
M_cs_load = M_cs .* diff([0,structure_xi_mid,1])' * 1./diff(fuse_xi);

% get relative chord position of structure nodes
r_structure = zeros(size(fuselage.geometry.border_pos));
[structure_xi_unique,idx_unique] = unique(structure_xi);
for i = 1:3
    r_structure(i,:) = interp1( structure_xi_unique, nodes_pos_sort(i,idx_unique), fuse_xi, 'linear', 'extrap' );
end
dist_vectors = fuselage.geometry.border_pos(3,:) - r_structure(3,:);
structure_height_rel = dist_vectors./(fuselage.geometry.width/2);
structure_height_rel_c = interp1(fuse_xi,structure_height_rel,fuse_xi_c);

% build global linear interpolation matrix
DOF = 6;
% T_ws = zeros(num_vortex,num_nodes_total*DOF);
T_bs = zeros(num_border*5,num_nodes_total*DOF);
T_cs = zeros(num_cntrl_pt*3,num_nodes_total*DOF);
T_as = zeros(num_cntrl_pt,num_nodes_total*DOF);
T_Bs = zeros(num_cntrl_pt,num_nodes_total*DOF);
T_sc = zeros(num_nodes_total*DOF,num_cntrl_pt*3);
T_b_x = zeros(num_border,num_nodes_total*DOF);
T_b_y = T_b_x;
T_b_z = T_b_x;
T_b_rotY = T_b_x;
T_b_rotZ = T_b_x;
T_c_x = zeros(num_cntrl_pt,num_nodes_total*DOF);
T_c_y = T_c_x;
T_c_z = T_c_x;
T_c_rotY = T_c_x;
T_c_rotZ = T_c_x;
T_f_x = zeros(num_nodes_total*DOF,num_cntrl_pt);
T_f_y = T_f_x;
T_f_z = T_f_x;


j = 1;
for i = 1:num_nodes_total
    if assignment_vector(i) ~= 0
        idx_T = find(idx_sort==j);
        idx = (i-1)*DOF;
        T_b_x(:,idx+1) = T_b(:,idx_T);
        T_b_y(:,idx+2) = T_b(:,idx_T);
        T_b_z(:,idx+3) = T_b(:,idx_T);
        T_b_rotY(:,idx+5) = T_b(:,idx_T);
        T_b_rotZ(:,idx+6) = T_b(:,idx_T);
        
        T_c_x(:,idx+1) = T_c(:,idx_T);
        T_c_y(:,idx+2) = T_c(:,idx_T);
        T_c_z(:,idx+3) = T_c(:,idx_T);
        T_c_rotY(:,idx+5) = T_c(:,idx_T);
        T_c_rotZ(:,idx+6) = T_c(:,idx_T);
        
        T_f_x(idx+1,:) = M_cs_load(idx_T,:);
        T_f_y(idx+2,:) = M_cs_load(idx_T,:);
        T_f_z(idx+3,:) = M_cs_load(idx_T,:);

        j = j+1;
    end
end
T_bs(1:5:end,:) = T_b_x;
T_bs(2:5:end,:) = T_b_y;
T_bs(3:5:end,:) = T_b_z;
T_bs(4:5:end,:) = T_b_rotY;
T_bs(5:5:end,:) = T_b_rotZ;


T_cs(1:3:end,:) = T_c_x;
T_cs(2:3:end,:) = T_c_y;
T_cs(3:3:end,:) = T_c_z;
T_as(1:1:end,:) = T_c_rotY;
T_Bs(1:1:end,:) = T_c_rotZ;

T_sc(:,1:3:end) = T_f_x;
T_sc(:,2:3:end) = T_f_y;
T_sc(:,3:3:end) = T_f_z;

if is_modal
    T_bsr = T_bs * structure.modal.T;
    T_csr = T_cs * structure.modal.T;
    T_asr = T_as * structure.modal.T;
    T_Bsr = T_Bs * structure.modal.T;
    T_scr = structure.modal.T' * T_sc;
else
    T_bsr = T_bs;
    T_csr = T_cs;
    T_asr = T_as;
    T_Bsr = T_Bs;
    T_scr = T_sc;
end

% aeroelasticity.T_vs_full = T_vs;
% aeroelasticity.T_cs_full = T_cs;
% aeroelasticity.T_sc_full = T_sc;

% aeroelasticity.T_bs = T_bsr;
% aeroelasticity.T_cs = T_csr;
% aeroelasticity.T_sc = T_scr;

fuselage.aeroelasticity.T_cs(:) = T_csr;
fuselage.aeroelasticity.T_as(:) = T_asr;
fuselage.aeroelasticity.T_Bs(:) = T_Bsr;
fuselage.aeroelasticity.T_sc(:) = T_scr;

end


function M = getLinInterpMatrix(x,xq)

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