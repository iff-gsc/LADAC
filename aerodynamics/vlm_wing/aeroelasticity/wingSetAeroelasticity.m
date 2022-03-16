function [aeroelasticity,T_csfr] = wingSetAeroelasticity( wingGeometry, structure, is_modal )
% wingSetAeroelasticity computes transformation matrices that for the 
% coupling of aerodynamics to structural dynamics
%   For the coupling of aerodynamics and structural dynamics some
%   transformations between the models are required. For instance the
%   aerodynamics load vector (3D forces and pitching moment at all panel 
%   locations) must be converted to structural load (usually in generalized
%   coordinates). Moreover, the structural displacements, velocity and
%   accelerations (usually in generalized coordinates) must be converted to
%   displacements, velocity and accelerations of the aerodynamics panels.
%   This function should be called one time during the initialization of
%   the model and obtained coupling should be considered to be constant.
%   This is valid considering a linear model (a nonlinear coupling would
%   significantly increase the computation time). For the linear coupling,
%   constant transformation matrices are required that map from the
%   structural vectors to the aerodynamics vectors and vice versa. These
%   transformation matrices are computed by this function.
%   The transformation matrices are filled based linear interpolation
%   method, i.e. this coupling considers linear interpolation between
%   structural quantities and aerodynamic quantities. This could be
%   extended to higher order interpolations in the future but the benefit
%   is probably not really noticeable. Also note that for the mapping of
%   displacements and moments, also the lever arm and rotations are
%   considered.
%   You will probably have a hard time to understand this code because it
%   is realatively complicated. However, note that this is no rocket science
%   but just a linear coupling as explained above. It just takes several
%   lines of code to implement. Surely the code could be improved but you
%   should not touch it unless you know what you are doing.
% 
% Inputs:
%   wingGeometry        wing geometry struct (see wingSetGeometryState)
%   structure           structure struct (see structureCreateFromNastran)
%                       or reduced order structure struct (see
%                       structureGetReduced)
%   is_model            logical that indicates whether a structure struct
%                       or a reduced structure struct is passed as second
%                       function parameter)
% 
% Outputs:
%   aeroelasticity      structure that contains the required transformation
%                       matrices (see wingCreateAeroelasticity)
%   T_csfr              additional transformation matrix that maps the
%                       generalized force vector of the structural dynamics
%                       model to the force and pitching moment vector of
%                       the aerodynamics model
% 
% See also:
%   wingSetGeometryState, structureCreateFromNastran, structureGetReduced,
%   wingCreateAeroelasticity
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% get wing part of structure and compute the spanwise coordinate (eta) for
% each node
chord = wingGeometry.vortex.c;
chord_c = wingGeometry.ctrl_pt.c;
r_vortex = wingGeometry.vortex.pos + wingGeometry.origin;
wing_25_local_line_length = vecnorm( diff( wingGeometry.vortex.pos.*[0;1;1],1,2),2,1 );
wing_25_total_line_length = sum( wing_25_local_line_length );
wing_eta = cumsum( [0,wing_25_local_line_length] / wing_25_total_line_length );
wing_eta_c = wing_eta(1:end-1) + diff(wing_eta)/2;

assignment_vector = wingAssignToStructure(wingGeometry,structure);

wing_idx = assignment_vector(assignment_vector~=0);
nodes_pos = structure.xyz(:,assignment_vector~=0);
num_nodes = sum(assignment_vector~=0);
num_nodes_total = length(assignment_vector);
num_vortex = length(wing_eta);
num_ctrl_pt = num_vortex-1;

[wing_idx_sort,idx_sort] = sort(wing_idx);
nodes_pos_sort = nodes_pos(:,idx_sort);

structure_eta = zeros(1,num_nodes);

for i = 1:num_nodes
    eta_left = wing_eta(wing_idx_sort(i));
    Delta_pos = norm(nodes_pos_sort(2:3,i)-r_vortex(2:3,wing_idx_sort(i)),2);
    Delta_pos_wing = wing_25_local_line_length(wing_idx_sort(i));
    Delta_eta = diff(wing_eta(wing_idx_sort(i):wing_idx_sort(i)+1));
    structure_eta(i) = eta_left + Delta_pos/Delta_pos_wing * Delta_eta;
end
% re-order (necessary for nodes at same panels)
[structure_eta,idx_sort_2] = sort(structure_eta);
wing_idx_sort = wing_idx_sort(idx_sort_2);
idx_sort = idx_sort(idx_sort_2);
nodes_pos_sort = nodes_pos_sort(:,idx_sort_2);
structure_eta_mid = structure_eta(1:end-1) + diff(structure_eta)/2;

% build local linear interpolation matrix (also extrapolation)
T_v = getLinInterpMatrix(structure_eta,wing_eta);
T_c = getLinInterpMatrix(structure_eta,wing_eta_c);
M_cs = getLinInterpMatrix(structure_eta,wing_eta_c);
M_sc = getLinInterpMatrix(wing_eta_c,structure_eta);
% force distribution must be equal but force per node depends on number of
% nodes (transpose of interpolation matrix assures that each force is
% considered once - because the sum of all columns is one)
M_sc_load = M_cs';
M_cs_load = M_sc';

% get relative chord position of structure nodes
r_structure = zeros(size(r_vortex));
[structure_eta_unique,idx_unique] = unique(structure_eta);
for i = 1:3
    r_structure(i,:) = interp1( structure_eta_unique, nodes_pos_sort(i,idx_unique), wing_eta, 'linear', 'extrap' );
end
dist_vectors = r_vortex([1,3],:) - r_structure([1,3],:);
dist = sign(max(dist_vectors(1,:))) .* vecnorm( dist_vectors, 2, 1 );
structure_chord_rel = 1/4 + dist./chord;
structure_chord_rel_c = interp1(wing_eta,structure_chord_rel,wing_eta_c);

% build global linear interpolation matrix
DOF = 6;
% T_ws = zeros(num_vortex,num_nodes_total*DOF);
T_vs = zeros(num_vortex*3,num_nodes_total*DOF);
T_cs = zeros(num_ctrl_pt*4,num_nodes_total*DOF);
T_sc_f = zeros(num_nodes_total*DOF,num_ctrl_pt*4);
T_cs_f = zeros(num_ctrl_pt*4,num_nodes_total*DOF);
T_v_x = zeros(num_vortex,num_nodes_total*DOF);
T_v_y = T_v_x;
T_v_z = T_v_x;
T_c_x = zeros(num_ctrl_pt,num_nodes_total*DOF);
T_c_y = T_c_x;
T_c_z = T_c_x;
T_c_twistY = T_c_x;
T_c_twistZ = T_c_x;
T_sc_f_x = zeros(num_nodes_total*DOF,num_ctrl_pt);
T_sc_f_y = T_sc_f_x;
T_sc_f_z = T_sc_f_x;
T_sc_f_m = T_sc_f_x;
T_sc_f_mz = T_sc_f_x;
T_sc_f_my = T_sc_f_x;
T_cs_f_x = zeros(num_ctrl_pt,num_nodes_total*DOF);
T_cs_f_y = T_cs_f_x;
T_cs_f_z = T_cs_f_x;
T_cs_f_m = T_cs_f_x;

% combine rotation about y and z (for structure) to spanwise twist
radialVectorYZ = cross( repmat([1;0;0],1,num_ctrl_pt), wingGetNormalVectorFromGeometry(wingGeometry) );
radialVectorYZ_v = zeros( 3, num_vortex );
for i = 1:3
    radialVectorYZ_v(i,:) = interp1( wing_eta_c, radialVectorYZ(i,:), wing_eta, 'linear', 'extrap' );
end
angleYZ = atan(radialVectorYZ(3,:)./radialVectorYZ(2,:));
% parts/factors are computed such that rotPartY + rotPartz = 1
% (not sure if this is correct - only relevant for dihedrals significantly
% different from 0deg and 90deg)
rotPartY = cos(angleYZ).^2;
rotPartZ = sign(angleYZ).*sin(angleYZ).^2;


j = 1;
for i = 1:num_nodes_total
    if assignment_vector(i) ~= 0
        idx_T = find(idx_sort==j);
        idx = (i-1)*DOF;
        T_v_x(:,idx+1) = T_v(:,idx_T);
        T_v_y(:,idx+2) = T_v(:,idx_T);
        T_v_z(:,idx+3) = T_v(:,idx_T);
        T_v_tmp = T_v(:,idx_T) .* -(structure_chord_rel'-0.25).*chord';
        T_v_z(:,idx+5) = T_v_tmp;
        T_v_y(:,idx+6) = -T_v_tmp;
        
        T_c_x(:,idx+1) = T_c(:,idx_T);
        T_c_y(:,idx+2) = T_c(:,idx_T);
        T_c_z(:,idx+3) = T_c(:,idx_T);
        T_c_tmp = T_c(:,idx_T) .* -(structure_chord_rel_c'-0.75).*chord_c';
        T_c_z(:,idx+5) = T_c_tmp;
        T_c_y(:,idx+6) = -T_c_tmp;
        T_c_twistY(:,idx+5) = T_c(:,idx_T) .* rotPartY';
        T_c_twistZ(:,idx+6) = T_c(:,idx_T) .* rotPartZ';
        
        T_sc_f_x(idx+1,:) = M_sc_load(idx_T,:);
        T_sc_f_y(idx+2,:) = M_sc_load(idx_T,:);
        T_sc_f_z(idx+3,:) = M_sc_load(idx_T,:);
        T_sc_f_m(idx+5,:) = M_sc_load(idx_T,:) .* rotPartY;
        T_sc_f_m(idx+6,:) = M_sc_load(idx_T,:) .* rotPartZ;
        T_sc_f_tmp = M_sc_load(idx_T,:) .* -(structure_chord_rel_c-0.25).*chord_c;
        T_sc_f_mz(idx+5,:) = T_sc_f_tmp;
        T_sc_f_my(idx+6,:) = -T_sc_f_tmp;
        
        T_cs_f_x(:,idx+1) = M_cs_load(:,idx_T);
        T_cs_f_y(:,idx+2) = M_cs_load(:,idx_T);
        T_cs_f_z(:,idx+3) = M_cs_load(:,idx_T);
        T_cs_f_m(:,idx+5) = M_cs_load(:,idx_T) .* rotPartY';
        T_cs_f_m(:,idx+6) = M_cs_load(:,idx_T) .* rotPartZ';
        T_cs_f_tmp = M_cs_load(:,idx_T) .* (structure_chord_rel_c'-0.25).*chord_c';
        T_cs_f_m(:,idx+2) = T_cs_f_tmp .* -rotPartZ';
        T_cs_f_m(:,idx+3) = T_cs_f_tmp .* rotPartY';

        j = j+1;
    end
end
T_vs(1:3:end,:) = T_v_x;
T_vs(2:3:end,:) = T_v_y;
T_vs(3:3:end,:) = T_v_z;

T_cs(1:4:end,:) = T_c_x;
T_cs(2:4:end,:) = T_c_y;
T_cs(3:4:end,:) = T_c_z;
T_cs(4:4:end,:) = T_c_twistY + T_c_twistZ;

T_sc_f(:,1:4:end) = T_sc_f_x;
T_sc_f(:,2:4:end) = T_sc_f_y + T_sc_f_my;
T_sc_f(:,3:4:end) = T_sc_f_z + T_sc_f_mz;
T_sc_f(:,4:4:end) = T_sc_f_m;

T_cs_f(1:4:end,:) = T_cs_f_x;
T_cs_f(2:4:end,:) = T_cs_f_y;
T_cs_f(3:4:end,:) = T_cs_f_z;
T_cs_f(4:4:end,:) = T_cs_f_m;

if is_modal
    T_vsr = T_vs * structure.modal.T;
    T_csr = T_cs * structure.modal.T;
    T_scr = structure.modal.T' * T_sc_f;
    T_csfr = T_cs_f * structure.modal.T;
else
    T_vsr = T_vs;
    T_csr = T_cs;
    T_scr = T_sc_f;
    T_csfr = T_cs_f;
end

% aeroelasticity.T_vs_full = T_vs;
% aeroelasticity.T_cs_full = T_cs;
% aeroelasticity.T_sc_full = T_sc;
aeroelasticity.T_vs = T_vsr;
aeroelasticity.T_cs = T_csr;
aeroelasticity.T_sc = T_scr;

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