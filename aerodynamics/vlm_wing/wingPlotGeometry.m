function [] = wingPlotGeometry( wing, dim, varargin )
%plotWingGeometry depicts the wing
%   plotWingGeometry displays the designed wing according to its definition
%   by the number of vortices and panels respectively, or by its
%   parameters. Moreover, the plot can either be a 3D- or a 2D-plot.
%
% Syntax: [] = plotWingGeometry( wing, dim, type, panel )
%
% Input:
%    wing                   A struct containing all inforamtion about the
%                           wing
%                           (struct)
%    dim                    Dimension, determines whether a 2D- or a
%                           3D-plot is created
%                           (double)
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2021 Lucas Schreer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% check value of dim
if dim ~= 3 && dim ~= 2
    error('Check entered dimension of plot. Only 2D- (2) and 3D-plots (3)!')
end

if ~isempty(varargin)
    FaceColor = varargin{1};
else
    FaceColor = 'y';
end

LineColor = [0.5,0.5,0.2];


% rotation matrices
M_incidence = [ ...
    cos(wing.geometry.rotation(2)), 0, sin(wing.geometry.rotation(2)); ...
    0, 1, 0; ...
    -sin(wing.geometry.rotation(2)), 0, cos(wing.geometry.rotation(2)) ...
    ];

M_rot_x = [ ...
    1, 0, 0; ...
    0, cos(wing.geometry.rotation(1)), sin(wing.geometry.rotation(1)); ...
    0, -sin(wing.geometry.rotation(1)), cos(wing.geometry.rotation(1)) ...
    ];

M = M_rot_x * M_incidence;

wing.geometry.line_25.pos = M' * wing.geometry.line_25.pos;
n_panel_x = 1;
for i = 1:n_panel_x
    wing.geometry.ctrl_pt.pos(:,:,i) = M' * wing.geometry.ctrl_pt.pos(:,:,i);
end


% get twist at vortex points (plus global incidence)
twist_cntrl = wing.geometry.ctrl_pt.local_incidence - wing.params.i;
twist_vortex = interp1(wing.geometry.ctrl_pt.pos(2,:),...
    wing.geometry.ctrl_pt.local_incidence - wing.params.i,...
    wing.geometry.line_25.pos(2,:),'pchip');

% set geometric features
quarter_chord = wing.geometry.origin(1) + wing.geometry.line_25.pos(1,:);
flap_depth_ext = [wing.geometry.segments.flap_depth(1),...
    wing.geometry.segments.flap_depth,...
    wing.geometry.segments.flap_depth(end)];
flap_defl_ext = deg2rad( [ wing.state.actuators.segments.pos(1,1), ...
    wing.state.actuators.segments.pos(1,:), ...
    wing.state.actuators.segments.pos(1,end) ] );
flap_depth = flap_depth_ext(1:end-1) + diff(flap_depth_ext)/2;
x_lead = wing.geometry.origin(1) + wing.geometry.line_25.pos(1,:) ...
    + wing.geometry.line_25.c / 4 .* cos(twist_vortex);
x_trail_left = wing.geometry.origin(1) + wing.geometry.line_25.pos(1,:) - ...
    wing.geometry.line_25.c .* (3/4 - flap_depth_ext(2:end))...
    .* cos(twist_vortex);
x_trail_right = wing.geometry.origin(1) + wing.geometry.line_25.pos(1,:) - ...
    wing.geometry.line_25.c .* (3/4 - flap_depth_ext(1:end-1))...
    .* cos(twist_vortex);
x_flap_left = x_trail_left - wing.geometry.line_25.c .* ...
    flap_depth_ext(2:end) .* cos(twist_vortex+flap_defl_ext(2:end));
x_flap_right = x_trail_right - wing.geometry.line_25.c .* ...
    flap_depth_ext(1:end-1) .* cos(twist_vortex+flap_defl_ext(1:end-1));
y = wing.geometry.line_25.pos(2,:);
z = wing.geometry.origin(3) + wing.geometry.line_25.pos(3,:);
z_lead = wing.geometry.origin(3) + wing.geometry.line_25.pos(3,:) ...
    - wing.geometry.line_25.c/4 .* sin(twist_vortex);
z_trail_left = wing.geometry.origin(3) + wing.geometry.line_25.pos(3,:) + ...
    wing.geometry.line_25.c .* (3/4 - flap_depth_ext(2:end))...
    .* sin(twist_vortex);
z_trail_right = wing.geometry.origin(3) + wing.geometry.line_25.pos(3,:) + ...
    wing.geometry.line_25.c .* (3/4 - flap_depth_ext(1:end-1))...
    .* sin(twist_vortex);
z_flap_left = z_trail_left + wing.geometry.line_25.c .* ...
    flap_depth_ext(2:end) .* sin(twist_vortex+flap_defl_ext(2:end));
z_flap_right = z_trail_right + wing.geometry.line_25.c .* ...
    flap_depth_ext(1:end-1) .* sin(twist_vortex+flap_defl_ext(1:end-1));

pos_quarter = [quarter_chord;y;z];
pos_lead = [x_lead;y;z_lead];
pos_trail_left = [x_trail_left;y;z_trail_left];
pos_trail_right = [x_trail_right;y;z_trail_right];
pos_flap_left = [x_flap_left;y;z_flap_left];
pos_flap_right = [x_flap_right;y;z_flap_right];

% M = eye(3);

pos_quarter = M * (pos_quarter - wing.geometry.origin) + wing.geometry.origin;
pos_lead = M * (pos_lead - wing.geometry.origin) + wing.geometry.origin;
pos_trail_left = M * (pos_trail_left - wing.geometry.origin) + wing.geometry.origin;
pos_trail_right = M * (pos_trail_right - wing.geometry.origin) + wing.geometry.origin;
pos_flap_left = M * (pos_flap_left - wing.geometry.origin) + wing.geometry.origin;
pos_flap_right = M * (pos_flap_right - wing.geometry.origin) + wing.geometry.origin;

quarter_chord(:) = pos_quarter(1,:);
y(:) = pos_quarter(2,:);
z(:) = pos_quarter(3,:);

x_lead(:) = pos_lead(1,:);
z_lead(:) = pos_lead(3,:);

x_trail_left(:) = pos_trail_left(1,:);
z_trail_left(:) = pos_trail_left(3,:);

x_trail_right(:) = pos_trail_right(1,:);
z_trail_right(:) = pos_trail_right(3,:);

x_flap_left(:) = pos_flap_left(1,:);
z_flap_left(:) = pos_flap_left(3,:);

x_flap_right(:) = pos_flap_right(1,:);
z_flap_right(:) = pos_flap_right(3,:);

% plot
plot3( quarter_chord, y,z, '--', 'Color', LineColor )
hold on
grid on


for i = 1:wing.n_panel
    % wing
    patch( ...
        [x_lead(i),x_lead(i+1),x_trail_right(i+1),x_trail_left(i),x_lead(i)], ...
        [y(i),y(i+1),y(i+1),y(i),y(i)], ...
        [z_lead(i),z_lead(i+1),z_trail_right(i+1),z_trail_left(i),z_lead(i)], ...
        FaceColor, 'EdgeColor','k' ...
        )
    % flap
    patch( ...
        [x_trail_left(i),x_trail_right(i+1),x_flap_right(i+1),x_flap_left(i),x_trail_left(i)], ...
        [y(i),y(i+1),y(i+1),y(i),y(i)], ...
        [z_trail_left(i),z_trail_right(i+1),z_flap_right(i+1),z_flap_left(i),z_trail_left(i)], ...
        [0.3,0.4,0.5], 'EdgeColor','k' ...
        )
end

wing.geometry.ctrl_pt.pos = M * wing.geometry.ctrl_pt.pos;
plot3( wing.geometry.ctrl_pt.pos(1,:) + wing.geometry.origin(1), ...
    wing.geometry.ctrl_pt.pos(2,:) + wing.geometry.origin(2), ...
    wing.geometry.ctrl_pt.pos(3,:) + wing.geometry.origin(3), 'bo')

axis equal

% labels
xlabel('x, in m')
ylabel('y, in m')
zlabel('z, in m')

% shade wing area
alpha(0.5);

hold off;

% perspective
if dim==3
    view(-150,25); % azimuth, elevation
else
    view(90,-90);
end

set(gca, 'YDir','reverse')
set(gca, 'ZDir','reverse')


end
