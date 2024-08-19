% This test script simulates dynamic stall
% Therefore, the angle of attack is varied with the following function:
%   alpha(t) = alpha_0 + Delta_alpha * sin(omega*t)

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

%% init Simulink model

% Mach number, -
Ma = 0.745;
% reduced frequency, -
k = 0.075;
% chord, m
c = 2.8;
% speed of sound, m/s
a = 300;
% mean angle of attack
alpha_0 = 4;
% angle of attack oscillation amplitude
Delta_alpha = 5;

% load analytic coefficient parameters for F15_bl (neural network weights)
F15 = airfoilAnalytic0515LoadParams('airfoilAnalytic0515_params_F15');
% compute analytic lift coefficient function parameters for the airfoil
fcl = airfoilAnalytic0515Ma( F15.wcl, Ma, F15.ncl, F15.ocl );
fcd = airfoilAnalytic0515Ma( F15.wcd, Ma, F15.ncd, F15.ocd );
fcm = airfoilAnalytic0515Ma( F15.wcm, Ma, F15.ncm, F15.ocm );
% airspeed, m/s
V = Ma*a;

% simulation time
t_sim = 5 * 1/(k*2*V/c/(2*pi));

%% simulate

sim('example_sim_airfoilDynStall');

%% plot

subplot(2,2,1)
plot( rad2deg(simout.Data(round(end/2):end,1)), simout.Data(round(end/2):end,2) )
hold on
plot( rad2deg(simout.Data(round(end/2):end,1)), simout.Data(round(end/2):end,3) )
plot( rad2deg(simout.Data(round(end/2):end,1)), simout.Data(round(end/2):end,4) )
grid on
legend('static','unsteady (potential flow)','unsteady (dynamic stall)','location','best')
xlabel('Angle of attack, deg')
ylabel('Lift coefficient, -')


subplot(2,2,2)
plot( rad2deg(simout.Data(round(end/2):end,1)), simout.Data(round(end/2):end,5) )
hold on
plot( rad2deg(simout.Data(round(end/2):end,1)), simout.Data(round(end/2):end,6) )
grid on
legend('static','unsteady (dynamic stall)','location','best')
xlabel('Angle of attack, deg')
ylabel('Drag coefficient, -')


subplot(2,2,3)
plot( rad2deg(simout.Data(round(end/2):end,1)), simout.Data(round(end/2):end,7) )
hold on
plot( rad2deg(simout.Data(round(end/2):end,1)), simout.Data(round(end/2):end,9) )
plot( rad2deg(simout.Data(round(end/2):end,1)), simout.Data(round(end/2):end,8) )
grid on
legend('static','unsteady (potential flow)','unsteady (dynamic stall)','location','best')
xlabel('Angle of attack, deg')
ylabel('Pitching moment coefficient, -')


subplot(2,2,4)
plot( rad2deg(simout.Data(round(end/2):end,1)), simout.Data(round(end/2):end,10) )
hold on
plot( rad2deg(simout.Data(round(end/2):end,1)), simout.Data(round(end/2):end,11) )
plot( rad2deg(simout.Data(round(end/2):end,1)), simout.Data(round(end/2):end,12) )
grid on
legend('static','effective','unsteady','location','best')
xlabel('Angle of attack, deg')
ylabel('TE separation point, -')
