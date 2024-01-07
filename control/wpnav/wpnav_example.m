
% Waypoints are extracted from:
% https://docs.px4.io/main/en/flight_modes_fw/mission.html
waypoints = [ ...
    0,0,0; 20,0,0; 40,0,0; 68,12,20; 74,20,10; 70,34,30; 60,60,40; 26,48,40; ...
    -36,28,10; 29,22,10; 1,12,10; -12,15,10; 24,36,10; 52,66,10; ...
    -2,58,10; 58,26,0 ...
    ]';

wp_radius = 12;

figure
wpnavPlot(waypoints,12,'dim',2);

figure
wpnavPlot(waypoints,12,'dim',3)