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
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2021 Lucas Schreer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

is_elliptical = false;

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
    end
end

% create planform wing geometry
geometry = geometryPlanform( params, n_panel, is_elliptical );

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

end



function geometry = geometryPlanform( params, n_panel, is_elliptical )

    geometry = wingInitGeometry( n_panel );

    n_vortex = n_panel + 1;
    
    % distance between control points = panel width, in m
    segment_width = 2*params.y_segments_wing(end) / n_panel;
    
    if params.is_symmetrical
        y_vortex = -params.y_segments_wing(end) + ((1:n_vortex) - 1) * segment_width;
        % y coordinate of control point of panel (in the middle of panel in y
        % direction), in m
        y_ctrl_pt = y_vortex(1:n_panel) + segment_width / 2;
        y_segments_wing = [ -flip(params.y_segments_wing(2:end)), params.y_segments_wing ];
        c_segments_wing = [ flip(params.c(2:end)), params.c ];
    else
        segment_width = segment_width / 2;
        y_vortex = params.y_segments_wing(1) + ((1:n_vortex) - 1) * segment_width;
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
    geometry.ctrl_pt.c(:) = interp1( y_vortex, geometry.vortex.c, y_ctrl_pt );
    
    x_vortex = -params.c(1)/4*ones(1,n_vortex);
    z_vortex = zeros(1,n_vortex);
    
    x_cntrl = -params.c(1)/4 - 0.5*geometry.ctrl_pt.c;
    z_cntrl = zeros(1,n_panel);
    
    geometry.vortex.pos(:) = [ x_vortex; y_vortex; z_vortex ];
    geometry.ctrl_pt.pos(:) = [ x_cntrl; y_ctrl_pt; z_cntrl ];     
    
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
        y_segments_device_2 = y_segments_device;
        section_type_2 = params.section_type;
        flap_depth_2 = params.flap_depth;
    end
    for segment_index = 1:length(y_segments_device_2)-1
        % find the corresponding panel indices
        indices_c = ( geometry.ctrl_pt.pos(2,:) >= y_segments_device_2(segment_index) ) ...
            &  ( geometry.ctrl_pt.pos(2,:) < y_segments_device_2(segment_index + 1) );
        % set trailing edge type to panel indices
        for j = 1:size(params.control_input_index,1)
            geometry.segments.control_input_index_local(j,indices_c) = params.control_input_index(j,segment_index);
        end
        geometry.segments.type_local(indices_c) = section_type_2(segment_index);
        geometry.segments.flap_depth(indices_c) = flap_depth_2(segment_index);
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
    
    geometry.vortex.pos(3,:) = geometry.vortex.pos(3,:) + ...
        interp1( y_segments_sym, z_segments_sym, geometry.vortex.pos(2,:) );
    
    geometry.ctrl_pt.pos(3,:) = geometry.ctrl_pt.pos(3,:) + ...
        interp1( geometry.vortex.pos(2,:), geometry.vortex.pos(3,:), geometry.ctrl_pt.pos(2,:) );

end

function geometry = geometryTwist( params, geometry )
    
    if params.is_symmetrical
        twist_segments_sym = [ params.epsilon, 0, params.epsilon ];
        y_segments_sym = [ -params.y_segments_wing(2:end), params.y_segments_wing ];
    else
        twist_segments_sym = [0,params.epsilon];
        y_segments_sym = params.y_segments_wing;
    end
    
    twist_vortex = interp1( y_segments_sym, twist_segments_sym, geometry.vortex.pos(2,:) );
    geometry.ctrl_pt.local_incidence(:) = interp1( geometry.vortex.pos(2,:), twist_vortex, geometry.ctrl_pt.pos(2,:) ) ...
        + params.i;
    
    
    for i = 1:length(geometry.ctrl_pt.local_incidence)
        geometry.ctrl_pt.pos(:,i) =  geometry.vortex.pos(:,i) + ...
            axisAngle( geometry.ctrl_pt.pos(:,i) - geometry.vortex.pos(:,i), ...
            geometry.vortex.pos(:,i+1) - geometry.vortex.pos(:,i), geometry.ctrl_pt.local_incidence(i)-params.i );
    end

end

function geometry = geometrySweep( params, geometry )
    
	x_segments = zeros( size( params.y_segments_wing ) );
    for i = 2:length(x_segments)
        x_segments(i) = x_segments(i-1) - tan( params.lambda(i-1) ) ...
            * ( params.y_segments_wing(i) - params.y_segments_wing(i-1) );
    end
    
    if params.is_symmetrical
        x_segments_sym = [ x_segments(2:end), x_segments ];
        y_segments_sym = [ -params.y_segments_wing(2:end), params.y_segments_wing ];
    else
        x_segments_sym = x_segments;
        y_segments_sym = params.y_segments_wing;
    end
    
    Delta_x_vortex = interp1( y_segments_sym, x_segments_sym, geometry.vortex.pos(2,:) );
    geometry.vortex.pos(1,:) = geometry.vortex.pos(1,:) + Delta_x_vortex;
    
    geometry.ctrl_pt.pos(1,:) = geometry.ctrl_pt.pos(1,:) + ...
        interp1( geometry.vortex.pos(2,:), Delta_x_vortex, geometry.ctrl_pt.pos(2,:) );

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

    % vortex
    geometry.vortex.pos(:) = M_incidence * M_rot_x * geometry.vortex.pos;

    % control points
    geometry.ctrl_pt.pos(:) = M_incidence * M_rot_x * geometry.ctrl_pt.pos;
    
    % origin
    geometry.origin(:) = [params.x;0;params.z];
    
    % rotation
    geometry.rotation(:) = [params.rot_x;params.i;0];
    
end
