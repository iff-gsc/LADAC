
% Waypoints are extracted from:
% https://docs.px4.io/main/en/flight_modes_fw/mission.html
waypoints = [ ...
    0,0,0; 20,0,0; 40,0,0; 68,12,20; 74,20,10; 70,34,30; 60,60,40; 26,48,40; ...
    -36,28,10; 29,22,10; 1,12,10; -12,15,10; 24,36,10; 52,66,10; ...
    -2,58,10; 58,26,0 ...
    ]';

wp_radius = 12;

figure
wpnavPlot(waypoints,wp_radius,'dim',2);

figure
wpnavPlot(waypoints,wp_radius,'dim',3);

%%

figure
wpnavPlot(waypoints,wp_radius,'dim',3);
hold on

res = 5;
f = [1,1.23,2.41]'*0.0321;
mag = 1;
time = 0;

wp_idx = 1;
stage = 1;

for i = 1:size(waypoints,2)-1
    for j = 1:res
        p = waypoints(:,i) + j/res * (waypoints(:,i+1)-waypoints(:,i)) + 4*mag*sin(2*pi*f*time);
        time = time + 1;
        [p_match,wp_idx,stage,t,d] = wpnavMatch(waypoints,wp_radius,wp_idx,stage,p);
        plot3(p_match(1),p_match(2),-p_match(3),'g.','MarkerSize',15)
        plot3(p(1),p(2),-p(3),'g.')
        plot3([p_match(1),p(1)],[p_match(2),p(2)],-[p_match(3),p(3)],'g-')
    end
end

