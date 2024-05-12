function [geometry] = wingSetGeometry(params, n_panel, varargin )
%setGeometry   computes all necessary coordinates and geometry parameters
%for each panel
%   The function setGeometry calculates all necessary geometry postions
%   of each panel and control points for the lifting line theory. This
%   mainly are the absolute positions of the panel borders and the control
%   points at 1/4 chord in an aircraft body coordinate system. Furthermore,
%   a sweepback angle, a wing twist, a dihedral angle and different kinds
%   of trailing edges (i.e. simple edges, flaps or ailerons) are taken into
%   account.
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
%
% Outputs:
%    geometry               A struct which contains all the computed
%                           geometry values and postions for the panels and
%                           vortexes
%                           (struct)
%
% See also: wingCreate
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2021 Lucas Schreer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

n_panel_x = 1;

is_elliptical = false;
spacing = 'constant';

for i = 1:length(varargin)
    if ~ischar(varargin{i})
        continue;
    end
    switch varargin{i}
        case 'is_elliptical'
            if islogical(varargin{i+1})
                is_elliptical(:) = varargin{i+1};
            else
                error('Invalid option for parameter is_elliptical.')
            end
        case 'spacing'
            spacing = varargin{i+1};
    end
end

% create planform wing geometry
geometry = geometryPlanform( params, n_panel, is_elliptical, n_panel_x, spacing );

if n_panel_x > 1
    geometry = geometryCamber( params, geometry );
end

% set segments
geometry = geometrySegments( params, geometry );

% apply dihedral deformation
geometry = geometryDihedral( params, geometry );

% apply twist
geometry = geometryTwist( params, geometry );

% apply sweep
geometry = geometrySweep( params, geometry );

% apply global positioning
geometry = geometryGlobal( params, geometry );

line_25_ctrl_pos = geometry.line_25.pos(:,1:end-1,:) + 0.5*diff(geometry.line_25.pos,1,2);
diff_ctrl_pos = line_25_ctrl_pos - geometry.ctrl_pt.pos;
geometry.ctrl_pt_lever = vecnorm( diff_ctrl_pos, 2 , 1 );
geometry.ctrl_pt_lever = geometry.ctrl_pt_lever .* sign( diff_ctrl_pos(1,:,:) );

end



function [geometry] = geometryPlanform( params, n_panel, is_elliptical, n_panel_x, spacing )

    geometry = wingInitGeometry( n_panel );

    n_vortex = n_panel + 1;
    
    eta_partitions = unique( [ params.eta_segments_wing, ...
        params.eta_segments_device ] );

    % the number of panels for each partition is defined as follows:
    % - detect the panel of the whole wing with the highest y-length
    % - divide this panel in the middle
    % - repeat until the desired number of panels in y direction is reached
    eta_partitions_ny = eta_partitions;
    if params.is_symmetrical
        n_panel_side = floor(n_panel/2);
    else
        n_panel_side = n_panel;
    end
    if ~strcmp(spacing,'constant')
        ny_vec = ones( 1, n_panel );
        while length(eta_partitions_ny) < n_panel_side + 1
            [max_diff,idx_max_diff] = max(diff(eta_partitions_ny));
            idx_max_diff_ny = max(find(eta_partitions<eta_partitions_ny(idx_max_diff+1)));
            ny_vec(idx_max_diff_ny) = ny_vec(idx_max_diff_ny) + 1;
            eta_partitions_ny = [eta_partitions_ny(1:idx_max_diff),mean(eta_partitions_ny(idx_max_diff:idx_max_diff+1)),eta_partitions_ny(idx_max_diff+1:end)];
        end
    else
        eta_partitions_ny = linspace(0,1,n_panel_side+1);
    end
    
    if params.is_symmetrical
        y_vortex = params.y_segments_wing(end) * [ -flip(eta_partitions_ny(2:end)), eta_partitions_ny ];
        segment_width = diff( y_vortex );
        % y coordinate of control point of panel (in the middle of panel in y
        % direction), in m
        y_ctrl_pt = y_vortex(1:n_panel) + segment_width / 2;
        y_segments_wing = [ -flip(params.y_segments_wing(2:end)), params.y_segments_wing ];
        c_segments_wing = [ flip(params.c(2:end)), params.c ];
    else
        y_vortex = params.y_segments_wing(end) * eta_partitions_ny ;
        segment_width = diff( y_vortex );
        % y_vortex = params.y_segments_wing(1) + ((1:n_vortex) - 1) * segment_width;
        y_ctrl_pt = y_vortex(1:n_panel) + segment_width / 2;
        y_segments_wing = params.y_segments_wing;
        c_segments_wing = params.c;
    end
    
    if is_elliptical
        geometry.vortex.c(:) = 8 /(pi * params.AR) * ...
            sqrt( (params.b/2)^2 - y_vortex.^2 );
    else
        geometry.vortex.c(:) = interp1( y_segments_wing, c_segments_wing, y_vortex );
    end
    c_mid = median(geometry.vortex.c);
    % avoid singularities
    geometry.vortex.c(geometry.vortex.c<0.01*c_mid) = 0.01 * c_mid;
    geometry.line_25.c = geometry.vortex.c;
    geometry.ctrl_pt.c(:) = repmat(interp1( y_vortex, geometry.vortex.c, y_ctrl_pt )/n_panel_x,1,1,n_panel_x);
    
    x_25 = -params.c(1)/4*ones(1,n_vortex);
    z_25 = zeros(1,n_vortex);
    geometry.line_25.pos = [x_25;y_vortex;z_25];
    
    z_vortex = zeros(1,n_vortex);
    z_cntrl = zeros(1,n_panel);
    for i = 1:n_panel_x+1
        x_vortex = x_25 + geometry.line_25.c/4 - geometry.line_25.c/n_panel_x*(i-1+0.25);
        geometry.vortex.pos(:,:,i) = [ x_vortex; y_vortex; z_vortex ];
        if i <= n_panel_x
            x_cntrl = x_25(1) + sum(geometry.ctrl_pt.c,3)/4 - sum(geometry.ctrl_pt.c,3)/n_panel_x*(i-1+0.75);
            geometry.ctrl_pt.pos(:,:,i) = [ x_cntrl; y_ctrl_pt; z_cntrl ];
        end
    end
    
end


function [geometry] = geometryCamber( params, geometry )

    geometry = geometry;
    
end


function geometry = geometrySegments( params, geometry )
    
    % determination of local flight control device type
    % local segment type
    y_segments_device = params.eta_segments_device * params.b/2;
    if params.is_symmetrical
        y_segments_device_2 = [ -flip(y_segments_device(2:end)), y_segments_device ];
        section_type_2 = [ flip(params.section_type), params.section_type ];
        flap_depth_2 = [ flip(params.flap_depth), params.flap_depth ];
    else
        y_segments_device_2 = 2*y_segments_device;
        section_type_2 = params.section_type;
        flap_depth_2 = params.flap_depth;
    end
    for segment_index = 1:length(y_segments_device_2)-1
        % find the corresponding panel indices
        indices_c = ( geometry.ctrl_pt.pos(2,:,1) >= y_segments_device_2(segment_index) ) ...
            &  ( geometry.ctrl_pt.pos(2,:,1) < y_segments_device_2(segment_index + 1) );
        % set trailing edge type to panel indices
        for j = 1:size(params.control_input_index,1)
            geometry.segments.control_input_index_local(j,indices_c) = params.control_input_index(j,segment_index);
        end
        geometry.segments.type_local(indices_c) = section_type_2(segment_index);
        geometry.segments.flap_depth(indices_c) = flap_depth_2(segment_index);
    end
    y_seg = params.b/2*params.eta_segments_wing;
    x_25 = zeros(size(y_seg));
    for i = 2:length(x_25)
        x_25(i) = x_25(i-1) - (y_seg(i)-y_seg(i-1)) * params.lambda(i-1);
    end
    flap_depth_unique = unique(flap_depth_2);
    flap_depth_unique(flap_depth_unique==0) = [];
    dist = vecnorm(geometry.ctrl_pt.pos(2:3,:),2,1);
    dist_max = norm(geometry.line_25.pos(2:3,end),2);
    rel_dist = dist/dist_max;
    for i = 1:length(flap_depth_unique)
        x_seg_hinge = x_25 + params.c/4 - (1-flap_depth_unique(i)).*params.c;
        flap_sweep_seg = atan(diff(-x_seg_hinge)./diff(y_seg));
        is_depth = geometry.segments.flap_depth == flap_depth_unique(i);
        for j = 1:length(params.eta_segments_wing)-1
            is_seg = rel_dist >= params.eta_segments_wing(j) & rel_dist < params.eta_segments_wing(j+1);
            is_match = is_seg & is_depth;
            geometry.segments.flap_sweep(is_match) = flap_sweep_seg(j);
        end
    end

end


function geometry = geometryDihedral( params, geometry )

	z_segments = zeros( size( params.y_segments_wing ) );
    for i = 2:length(z_segments)
        z_segments(i) = z_segments(i-1) - tan( params.nu(i-1) ) ...
            * ( params.y_segments_wing(i) - params.y_segments_wing(i-1) );
    end
    
    if params.is_symmetrical
        z_segments_sym = [ z_segments(2:end), z_segments ];
        y_segments_sym = [ -params.y_segments_wing(2:end), params.y_segments_wing ];
    else
        z_segments_sym = z_segments;
        y_segments_sym = params.y_segments_wing;
    end
    
    n_panel_x = size(geometry.ctrl_pt.pos,3);
    
    for i = 1:n_panel_x+1
        geometry.vortex.pos(3,:,i) = geometry.vortex.pos(3,:,i) + ...
            interp1( y_segments_sym, z_segments_sym, geometry.vortex.pos(2,:,i) );
        if i <= n_panel_x
            geometry.ctrl_pt.pos(3,:,i) = geometry.ctrl_pt.pos(3,:,i) + ...
                interp1( geometry.vortex.pos(2,:,i), geometry.vortex.pos(3,:,i), geometry.ctrl_pt.pos(2,:,i) );   
        end
    end
    geometry.line_25.pos(3,:) = geometry.line_25.pos(3,:) + ...
        interp1( y_segments_sym, z_segments_sym, geometry.line_25.pos(2,:) );

end

function geometry = geometryTwist( params, geometry )
    
    if params.is_symmetrical
        twist_segments_sym = [ flip(params.epsilon), 0, params.epsilon ];
        c_segments_sym = [ flip(params.c), params.c(2:end) ];
        y_segments_sym = [ flip(-params.y_segments_wing(2:end)), params.y_segments_wing ];
    else
        twist_segments_sym = [0,params.epsilon];
        c_segments_sym = params.c;
        y_segments_sym = params.y_segments_wing;
    end
    
    pos_le_segments = interp1( geometry.vortex.pos(2,:,1), geometry.vortex.pos(:,:,1)', y_segments_sym )' ...
        + [ c_segments_sym/4.*cos(twist_segments_sym); zeros(size(c_segments_sym)); -c_segments_sym/4.*sin(twist_segments_sym) ];
    
    pos_te_segments = interp1( geometry.vortex.pos(2,:,1), geometry.vortex.pos(:,:,1)', y_segments_sym )' ...
        - [ c_segments_sym*3/4.*cos(twist_segments_sym); zeros(size(c_segments_sym)); -c_segments_sym*3/4.*sin(twist_segments_sym) ];
    
    pos_le = interp1( y_segments_sym, pos_le_segments', geometry.vortex.pos(2,:,1) )';
    pos_te = interp1( y_segments_sym, pos_te_segments', geometry.vortex.pos(2,:,1) )';
    
    % linear edges
    twist_vortex = asin( divideFinite(pos_te(3,:)-pos_le(3,:), vecnorm(pos_te-pos_le,2,1) ) );
    
    % linear twist angle
%     twist_vortex = interp1( y_segments_sym, twist_segments_sym, geometry.vortex.pos(2,:,1) );
    
    n_panel = size(geometry.ctrl_pt.pos,2);
    n_panel_x = size(geometry.ctrl_pt.pos,3);
    for i = 1:n_panel+1
        for j = 1:n_panel_x+1
            if i<=n_panel && j<=n_panel_x
                geometry.ctrl_pt.local_incidence(1,:,j) = interp1( geometry.vortex.pos(2,:,j), twist_vortex, geometry.ctrl_pt.pos(2,:,j) ) ...
                    + params.i;
                geometry.ctrl_pt.pos(:,i,j) = geometry.line_25.pos(:,i) + ...
                    axisAngle( geometry.ctrl_pt.pos(:,i,j) - geometry.line_25.pos(:,i), ...
                    geometry.line_25.pos(:,i+1) - geometry.line_25.pos(:,i), geometry.ctrl_pt.local_incidence(1,i,j)-params.i );
            end
            jj = min(j,n_panel_x);
            if i == 1
                rot_line = geometry.line_25.pos(:,i+1) - geometry.line_25.pos(:,i);
                rot_angle = geometry.ctrl_pt.local_incidence(1,i,jj);
            elseif i == n_panel+1
                rot_line = geometry.line_25.pos(:,i) - geometry.line_25.pos(:,i-1);
                rot_angle = geometry.ctrl_pt.local_incidence(1,i-1,jj);
            else
                rot_line = geometry.line_25.pos(:,i+1) - geometry.line_25.pos(:,i-1);
                rot_angle = 0.5*( geometry.ctrl_pt.local_incidence(1,i-1,jj) ...
                    + geometry.ctrl_pt.local_incidence(1,i,jj) );
            end
            geometry.vortex.pos(:,i,j) = geometry.line_25.pos(:,i) + ...
                axisAngle( geometry.vortex.pos(:,i,j) - geometry.line_25.pos(:,i), ...
                rot_line, rot_angle-params.i );
        end
    end

end

function geometry = geometrySweep( params, geometry )
    
	x_segments = zeros( size( params.y_segments_wing ) );
    for i = 2:length(x_segments)
        x_segments(i) = x_segments(i-1) - tan( params.lambda(i-1) ) ...
            * ( params.y_segments_wing(i) - params.y_segments_wing(i-1) );
    end
    
    if params.is_symmetrical
        x_segments_sym = [ flip(x_segments(2:end)), x_segments ];
        y_segments_sym = [ -flip(params.y_segments_wing(2:end)), params.y_segments_wing ];
    else
        x_segments_sym = x_segments;
        y_segments_sym = params.y_segments_wing;
    end
    n_panel_x = size(geometry.ctrl_pt.pos,3);
    for j = 1:n_panel_x + 1
        Delta_x_vortex = interp1( y_segments_sym, x_segments_sym, geometry.vortex.pos(2,:,j),'linear','extrap' );
        geometry.vortex.pos(1,:,j) = geometry.vortex.pos(1,:,j) + Delta_x_vortex;
        if j <= n_panel_x
            geometry.ctrl_pt.pos(1,:,j) = geometry.ctrl_pt.pos(1,:,j) + ...
                interp1( geometry.vortex.pos(2,:,j), Delta_x_vortex, geometry.ctrl_pt.pos(2,:,j),'linear','extrap' );
        end
    end
    Delta_x_line_25 = interp1( y_segments_sym, x_segments_sym, geometry.line_25.pos(2,:),'linear','extrap' );
    geometry.line_25.pos(1,:) = geometry.line_25.pos(1,:) + Delta_x_line_25;
end

function geometry = geometryGlobal( params, geometry )

    % rotation matrices
    M_incidence = [ ...
        cos(params.i), 0, sin(params.i); ...
        0, 1, 0; ...
        -sin(params.i), 0, cos(params.i) ...
        ];
    
    M_rot_x = [ ...
        1, 0, 0; ...
        0, cos(params.rot_x), sin(params.rot_x); ...
        0, -sin(params.rot_x), cos(params.rot_x) ...
        ];

    n_panel_x = size( geometry.ctrl_pt.pos, 3 );
    
    for j = 1:n_panel_x+1
        
        % vortex
        geometry.vortex.pos(:,:,j) = M_incidence * M_rot_x * geometry.vortex.pos(:,:,j);
        
        if j <= n_panel_x
            % control points
            geometry.ctrl_pt.pos(:,:,j) = M_incidence * M_rot_x * geometry.ctrl_pt.pos(:,:,j);
        end
        
    end
    
    geometry.line_25.pos = M_incidence * M_rot_x * geometry.line_25.pos;
    
    % origin
    geometry.origin(:) = [params.x;0;params.z];

    % rotation
    geometry.rotation(:) = [params.rot_x;params.i;0];
    
end
