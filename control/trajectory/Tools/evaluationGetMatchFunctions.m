function [pos, T, B, N] = evaluationGetMatchFunctions(traj, velo, g)
% trajPlot visualizes a trajectory struct.
%
% Inputs:
%   traj            trajectory struct, see trajInit
%
%   velo            constant trajectory tangent velocity
%                   (scalar), in m/s
%
%   g               gravitational earth acceleration
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

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Fabian Guecker
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% evaluate positions
resolution = 30;
pos = zeros(3,traj.num_sections_set*resolution);
pos_id_t = zeros(2,traj.num_sections_set*resolution);
waypoints = zeros(3,traj.num_sections_set+1);
traj_section = trajGetSection(traj,1);
waypoints(:,1) = trajSectionGetPos(traj_section,0);

t=linspace(0, 1, resolution);

for k=1:traj.num_sections_set
    
    traj_section = trajGetSection(traj,k);
    for j = 1:resolution
        idx = (k-1)*resolution+j;
        pos(:,idx) = trajSectionGetPos(traj_section,t(j));
        pos_id_t(:,idx) = [k; t(j)];
    end
    waypoints(:,k+1) = trajSectionGetPos(traj_section,1);
    
end

% plot
title('path from waypoints')
plot3(waypoints(1,:),waypoints(2,:),-waypoints(3,:),'ko')
hold on
plot3(pos(1,:),pos(2,:),-pos(3,:),'k-','linewidth',0.75)
hold off
xlabel('x in m')
ylabel('y in m')
zlabel('z in m')
legend('waypoints','trajectory', 'Location', 'northeast','AutoUpdate','off');
%grid
axis equal

%% Plot velocity
% perform matching for all vehicle positions
active_section = 1;

T = zeros(size(pos));
B = zeros(size(pos));
N = zeros(size(pos));

tic
hold on
%for i = 1:size(pos,2)
for i = 1:size(pos,2)
    % current vehicle position
    pos_ac = pos(:,i);
    
    %if i == 2
        
    % matching
    %[active_section, ~, t] = trajGetMatch(traj, pos_ac, active_section);
    %[active_section, ~, t] = trajGetMatchEnhanced(traj, pos_ac, active_section);
    %[active_section, ~, t] = trajGetMatchLaguerre(traj, pos_ac, active_section);
    %[active_section, ~, t] = trajGetMatchDotProd(traj, pos_ac, active_section);
    %[active_section, ~, t] = trajGetMatchDotProdGlobalOpt(traj, pos_ac, active_section);
    [active_section, ~, t] = trajGetMatchNewton(traj, pos_ac, active_section);
    
        
    R_turn = 10;
    T_vec = [1; 0; 0];
    
    %[active_section, ~, t] = trajGetMatchCustom(traj, pos_ac, active_section, R_turn, T_vec);
    
    traj_sec_m = trajGetSection(traj, active_section);
    pos_m = trajSectionGetPos(traj_sec_m, t);
    error_m = norm(pos_ac - pos_m);
    
     if(error_m > 1e-3)
         disp( [num2str(i),' Error: ', num2str(error_m),' m (', num2str(active_section), ' /  ', num2str(t), ')'] )
         disp(pos_id_t(:,i))
         
         plot3(pos_ac(1),pos_ac(2),-pos_ac(3),'kx','MarkerSize',12)
         plot3(pos_m(1),pos_m(2),-pos_m(3),'rx','MarkerSize',12)
         plot3([pos_ac(1);pos_m(1)],[pos_ac(2);pos_m(2)],-[pos_ac(3);pos_m(3)],'r-','linewidth',1.5)  
     end
    
    % compute matched position
    traj_section = trajGetSection(traj,active_section);
    
    [T(:,i),B(:,i),N(:,i)] = trajSectionGetFrenetSerretWithGravity(traj_section, velo, g, t);
     
    %disp(i)
end
 hold off
time = toc;
disp([num2str(time/size(pos,2)*1000), ' ms pro Sample'])

red = 5;

hold on
quiver3(pos(1,1:red:end),pos(2,1:red:end),-pos(3,1:red:end),T(1,1:red:end),T(2,1:red:end),-T(3,1:red:end),0.1,'r','LineWidth',2)
quiver3(pos(1,1:red:end),pos(2,1:red:end),-pos(3,1:red:end),B(1,1:red:end),B(2,1:red:end),-B(3,1:red:end),0.1,'g','LineWidth',2)
quiver3(pos(1,1:red:end),pos(2,1:red:end),-pos(3,1:red:end),N(1,1:red:end),N(2,1:red:end),-N(3,1:red:end),0.1,'b','LineWidth',2)
hold off

toc

end