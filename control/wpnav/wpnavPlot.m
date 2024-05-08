function [] = wpnavPlot( waypoints, wp_radius, varargin )
% wpnavPlot plot the specified waypoint track with line and circle segments
% 
% Syntax:
%   wpnavPlot( waypoints, wp_radius )
%   wpnavPlot( waypoints, wp_radius, 'dim', dim )
% 
% Inputs:
%   waypoints           Waypoints (3xN array, where N is the number of
%                       wapyoints) in g frame (north-east-down), in m
%   wp_radius           Waypoint radius (scalar), in m
%   dim                 Dimension (2D or 3D) (scalar): 2 for 2D, 3 for 3D
% 
% Outputs:
%   -
% 
% See also:
%   wpnavMatch, wpnavCircSeg

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2024 Yannic Beyer
%   Copyright (C) 2024 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

grey = [0.8,0.8,0.8];

dim = 2;
for i = 1:length(varargin)
    if strcmp(varargin{i},'dim')
        dim = varargin{i+1};
    end
end

if dim == 2
    plot(waypoints(1,:),waypoints(2,:),'k--o')
    hold on
    plot(waypoints(1,1),waypoints(2,1),'g*')
    plot(waypoints(1,end),waypoints(2,end),'r*')
    plot(waypoints(1,[end,1]),waypoints(2,[end,1]),'LineStyle','--','Color',grey)
elseif dim == 3
    plot3(waypoints(1,:),waypoints(2,:),-waypoints(3,:),'k--o')
    hold on
    plot3(waypoints(1,1),waypoints(2,1),-waypoints(3,1),'g*')
    plot3(waypoints(1,end),waypoints(2,end),-waypoints(3,end),'r*')
    plot3(waypoints(1,[end,1]),waypoints(2,[end,1]),-waypoints(3,[end,1]),'LineStyle','--','Color',grey)
    view(-37.5,30)
end

res = 100;
ang = linspace(0,2*pi,res);
num_wp = size(waypoints,2);
for i = 1:num_wp
    if i == 1
        circ_seg = wpnavCircSeg( waypoints(:,[end,1,2]), wp_radius );
        circ_color = grey;
    elseif i>1 && i<num_wp
        circ_seg = wpnavCircSeg( waypoints(:,i-1:i+1), wp_radius );
        circ_color = 'c';
    elseif i == num_wp
        circ_seg = wpnavCircSeg( waypoints(:,[end-1,end,1]), wp_radius );
        circ_color = grey;
    end
    alpha_s = linspace(0,circ_seg.angle,res);
    circ = zeros(3,res);
    for j = 1:res
        t = divideFinite( alpha_s(j), circ_seg.angle );
        circ(:,j) = wpnavCircSegGetPos( circ_seg, t );
    end

    if dim == 2
        plot(circ(1,:),circ(2,:),'LineStyle','-','Color',circ_color)
    elseif dim == 3
        plot3(circ(1,:),circ(2,:),-circ(3,:),'LineStyle','-','Color',circ_color)
    end
    wp_rad = circ_seg.wp_rad;
    
    if dim == 2
        x = wp_rad*cos(ang);
        y = wp_rad*sin(ang);
        plot(waypoints(1,i)+x,waypoints(2,i)+y,'b-')
    end

end

grid on
box on
axis equal

xlabel('North, m')
ylabel('East, m')


if dim == 3
    view(-130,30); % azimuth, elevation
    zlabel('Altitude, m')
    set(gca, 'YDir','reverse')
else
    view(90,-90);
end

% set(gca, 'ZDir','reverse')

hold off

end
