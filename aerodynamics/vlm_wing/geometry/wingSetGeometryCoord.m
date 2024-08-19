function [geometry] = wingSetGeometryCoord(params, n_panel, spacing )
%setGeometryCoord   computes all necessary coordinates and geometry 
% parameters for each panel
%   The function setGeometryCoord calculates all necessary geometry 
%   postions of each panel and control points for the lifting line theory.
%   In contrast to setGeometry, the section coordinates are used instead of
%   angles (sweep and dihedral). The advantage is, that there is no
%   singularity if a wing section points spanwise coordinate is
%   perpendicular to the y-axis of the aircraft.
%
% Syntax:  [geometry] = setGeometry(params, n_panel, spacing )
%
% Literature
%   [1] Nickel, K.; Wohlfahrt, M.: "Schwanzlose Flugzeuge: Ihre Auslegung
%       und ihre Eigenschaften"; Basel; Boston; Berlin: Birkhaeuser Verlag,
%       1990
%
% Inputs:
% 	 params                 A struct containing all parameters and key
%                           characteristics of the wing
%                           (struct)
%    n_panel                The number of panels into which the wing is
%                           divided
%                           (double)
%   spacing                 Specify the spacing of the panel by a char.
%                           Currently only 'constant' is supported.
%
% Outputs:
%    geometry               A struct which contains all the computed
%                           geometry values and postions for the panels and
%                           vortexes
%                           (struct)

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

n_panel_x = 1;

wing_25_local_line_length = vecnorm( diff(params.xyz_25.*[0;1;1],1,2),2,1 );
wing_25_total_line_length = sum( wing_25_local_line_length );


delta_alpha_1 = [0,params.epsilon];

xyz_25_local = params.xyz_25 - [params.x;0;params.z];
xyz_75_local = params.xyz_75 - [params.x;0;params.z];

control_input_index = params.control_input_index;

if params.is_symmetrical
    xyz_25  = [[1;-1;1].*flip(xyz_25_local(:,2:end),2),xyz_25_local];
    xyz_75  = [[1;-1;1].*flip(xyz_75_local(:,2:end),2),xyz_75_local];
    c       = [flip(params.c(2:end)),params.c];
    delta_alpha = params.i + [flip(delta_alpha_1(2:end)),delta_alpha_1];
    eta_length = 2;
    eta_25 = [ flip( -cumsum( wing_25_local_line_length ) ), 0, cumsum( wing_25_local_line_length ) ] / wing_25_total_line_length;
    eta_segments_device = [ -flip( params.eta_segments_device(2:end) ), params.eta_segments_device ];
    section_type = [ flip( params.section_type ), params.section_type ];
    flap_depth = [ flip( params.flap_depth ), params.flap_depth ];
else
    xyz_25 = xyz_25_local;
    xyz_75 = xyz_75_local;
    c       = params.c;
    delta_alpha = params.i + delta_alpha_1;
    eta_length = 1;
    eta_25 = [ 0, cumsum( wing_25_local_line_length ) ] / wing_25_total_line_length;
    eta_segments_device = params.eta_segments_device;
    section_type = params.section_type;
    flap_depth = params.flap_depth;
end

switch spacing
    case 'constant'
        delta_eta_vortex = eta_length / n_panel;
        eta_vortex = (1-eta_length):delta_eta_vortex:1;
        eta_ctrl_pt = eta_vortex(2:end)-delta_eta_vortex/2;
    case 'like_chord'
        delta_eta_vortex = eta_length / n_panel;
        eta_vortex = (1-eta_length):delta_eta_vortex:1;
        while true     
            eta_ctrl_pt = eta_vortex(2:end)-delta_eta_vortex/2;
            c_ctrl_pt = interp1( eta_25, c, eta_ctrl_pt );
            delta_eta_vortex_new = c_ctrl_pt / mean(c_ctrl_pt);
            % normalize so that the sum is 1 (don't manipulate span)
            delta_eta_vortex_new = delta_eta_vortex_new / (sum(delta_eta_vortex_new)/(eta_vortex(end)-eta_vortex(1)));
            eta_error = max(abs(delta_eta_vortex_new-delta_eta_vortex));
            delta_eta_vortex = delta_eta_vortex_new;
            eta_vortex(2:end-1) = cumsum(delta_eta_vortex(1:end-1))+eta_vortex(1);
            % problems occur, if the last value is slightly greater 1
            if eta_error < 1e-3*1/n_panel
                break;
            end
        end
end


origin = [params.x;0;params.z];

vortex.pos = zeros( 3, size(eta_vortex,2) );
vortex.pos(1,:) = interp1( eta_25, xyz_25(1,:), eta_vortex );
vortex.pos(2,:) = interp1( eta_25, xyz_25(2,:), eta_vortex );
vortex.pos(3,:) = interp1( eta_25, xyz_25(3,:), eta_vortex );
    
vortex.c = interp1( eta_25, c, eta_vortex );

incidence_vortex = interp1( eta_25, delta_alpha, eta_vortex );
for i = 2:n_panel_x + 1
    vortex.pos(1,:,i) = vortex.pos(1,:,i-1) - vortex.c .* cos(incidence_vortex);
    vortex.pos(2,:,i) = vortex.pos(2,:,i-1);
    vortex.pos(3,:,i) = vortex.pos(3,:,i-1) + vortex.c .* sin(incidence_vortex);
end

ctrl_pt.pos = zeros(3, size(eta_ctrl_pt,2) );
ctrl_pt.pos(1,:) = interp1( eta_25, xyz_75(1,:), eta_ctrl_pt );
ctrl_pt.pos(2,:) = interp1( eta_25, xyz_75(2,:), eta_ctrl_pt );
ctrl_pt.pos(3,:) = interp1( eta_25, xyz_75(3,:), eta_ctrl_pt );
ctrl_pt.c = interp1( eta_25, c, eta_ctrl_pt );
ctrl_pt.local_incidence = interp1( eta_25, delta_alpha, eta_ctrl_pt );

segments.control_input_index_local = zeros(size(control_input_index,1),size(eta_ctrl_pt,2));
segments.type_local = zeros(size(eta_ctrl_pt));
for i = 1:length(eta_segments_device)-1
    indices = eta_ctrl_pt >= eta_segments_device(i) & eta_ctrl_pt < eta_segments_device(i+1);
    for j = 1:size(control_input_index,1)
        segments.control_input_index_local(j,indices) = control_input_index(j,i);
    end
    segments.type_local(indices) = section_type(i);
    segments.flap_depth(indices) = flap_depth(i);
end

% flap sweep angle
segments.flap_sweep = zeros(size(segments.flap_depth));
for i = 1:length(segments.flap_sweep)
    pos1 = vortex.pos(:,i,1) + [-1;0;0]*(vortex.c(i)*(3/4-segments.flap_depth(i)));
    pos2 = vortex.pos(:,i+1,1) + [-1;0;0]*(vortex.c(i+1)*(3/4-segments.flap_depth(i)));
    segments.flap_sweep(i) = abs( atan((pos2(1)-pos1(1))/norm(pos2(2:3)-pos1(2:3),2)) );
end


geometry.origin = origin;
geometry.vortex = vortex;
geometry.ctrl_pt = ctrl_pt;
geometry.segments = segments;
geometry.rotation = [params.rot_x;params.i;0];

line_25_ctrl_pos = geometry.vortex.pos(:,1:end-1,1) + 0.5*diff(geometry.vortex.pos(:,:,1),1,2);
diff_ctrl_pos = line_25_ctrl_pos - geometry.ctrl_pt.pos;
geometry.ctrl_pt_lever = vecnorm( diff_ctrl_pos, 2 , 1 );
geometry.ctrl_pt_lever = geometry.ctrl_pt_lever .* sign( diff_ctrl_pos(1,:,:) );

geometry.line_25.c = geometry.vortex.c;
geometry.line_25.pos = geometry.vortex.pos(:,:,1);
end
