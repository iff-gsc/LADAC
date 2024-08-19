function [] = simpleWingPlotGeometry( wing, varargin )
% simpleWingPlotGeometry plots the contour as well as the neutral point
%   positions and the control point positions of a simple wing struct.
% 
% Inputs:
%   wing            simple wing struct (see wimpleWingLoadParams)
% 
% Outputs:
%   -
% 

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

alpha_M = 0;
beta_M = 0;
if ~isempty(varargin)
    alpha_M = varargin{1};
end
if length(varargin) > 1
    beta_M = varargin{2};
end

LineColor = [0.5,0.5,0.2];


% plot neutral points
xyz_np = simpleWingGetNeutralPoint(wing,alpha_M,beta_M);
plot3( xyz_np(1,:), xyz_np(2,:), xyz_np(3,:), 'x' )

hold on

% plot control points
plot3( wing.xyz_cntrl_pt_wing(1,:), wing.xyz_cntrl_pt_wing(2,:), wing.xyz_cntrl_pt_wing(3,:), 'o' )

% plot wing contour
c_i = 2*wing.geometry.c/(wing.geometry.z+1); % inner chord
c_o = c_i * wing.geometry.z; % outer chord
front_left = [ ...
    c_o/4 - wing.geometry.b/2*tan(wing.geometry.phi); ...
    -wing.geometry.b/2; ...
    -wing.geometry.b/2*sin(wing.geometry.v) ...
    ];
back_left = [ ...
    front_left(1) - c_o; ...
    front_left(2); ...
    front_left(3) ...
    ];    
front_center = [ ...
    c_i/4; ...
    0;
    0 ...
    ];
back_center = [ ...
    front_center(1) - c_i; ...
    front_center(2);
    front_center(3) ...
    ];

line( ...
    [front_left(1),front_center(1),front_left(1),back_left(1),back_center(1),back_left(1),front_left(1)], ...
    [front_left(2),front_center(2),-front_left(2),-back_left(2),back_center(2),back_left(2),front_left(2)], ...
    [front_left(3),front_center(3),front_left(3),back_left(3),back_center(3),back_left(3),front_left(3)], ...
    'Color',LineColor...
    )

hold off

legend('neutral points','control points','contour')

axis equal

set(gca, 'YDir','reverse')
set(gca, 'ZDir','reverse')

end