function [pos, T, B, N] = trajPlot(traj, velo, g)
% trajPlot visualizes a trajectory struct.
%   
% Inputs:
%   traj            trajectory struct, see trajInit
%
%   velo            constant trajectory tangent velocity
%                   (scalar), in m/s
%
%   g               gravitational earth acceleration (e.g. g = 9.81)
%                   (scalar), in m/s^2
% 
% Outputs:
%   pos             discrete sampled points of the trajectory
%                   (3xN vector), in m
%
%   T               tangential vector of the Frenet–Serret-Frame
%                   (3xN vector), dimensionless
%
%   B               binormal vector of the Frenet–Serret-Frame
%                   (3xN vector), dimensionless
%
%   N               normal vector of the Frenet–Serret-Frame
%                   (3xN vector), dimensionless
%
%
% Syntax: 
%   [pos, T, B, N] = trajPlot(traj, velo, g)
%
% See also: trajCreateFromWaypoints, trajSection_CubicSpline, trajInit
%

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Fabian Guecker
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% evaluate positions
resolution = 30;
pos = zeros(3,traj.num_sections_set*resolution);
waypoints = zeros(3,traj.num_sections_set+1);
traj_section = trajGetSection(traj,1);

waypoints(:,1) = trajSectionGetPos(traj_section,0);

T = zeros(size(pos));
B = zeros(size(pos));
N = zeros(size(pos));

for k=1:traj.num_sections_set
    
    t=linspace(0,1,resolution);
    traj_section = trajGetSection(traj,k);
    for j = 1:resolution
        idx = (k-1)*resolution+j;
        pos(:,idx) = trajSectionGetPos(traj_section,t(j));
        [T(:,idx),B(:,idx),N(:,idx)] = trajSectionGetFrenetSerretWithGravity(traj_section, velo, g, t(j));   
    end
    waypoints(:,k+1) = trajSectionGetPos(traj_section,1);    
    
end

red = 5;

% plot
title('path from waypoints')
plot3(waypoints(1,:),waypoints(2,:),-waypoints(3,:),'ko')
hold on
plot3(pos(1,:),pos(2,:),-pos(3,:),'k-','linewidth',0.75)
quiver3(pos(1,1:red:end),pos(2,1:red:end),-pos(3,1:red:end),T(1,1:red:end),T(2,1:red:end),-T(3,1:red:end),0.1,'r','LineWidth',2)
quiver3(pos(1,1:red:end),pos(2,1:red:end),-pos(3,1:red:end),B(1,1:red:end),B(2,1:red:end),-B(3,1:red:end),0.1,'g','LineWidth',2)
quiver3(pos(1,1:red:end),pos(2,1:red:end),-pos(3,1:red:end),N(1,1:red:end),N(2,1:red:end),-N(3,1:red:end),0.1,'b','LineWidth',2)
hold off
xlabel('x in m')
ylabel('y in m')
zlabel('z in m')
legend('waypoints','trajectory','tangent vector','bi-normal vector','normal vector', 'Location', 'northeast','AutoUpdate','off');
%grid
axis equal

set(gca,'Ydir','reverse');

end