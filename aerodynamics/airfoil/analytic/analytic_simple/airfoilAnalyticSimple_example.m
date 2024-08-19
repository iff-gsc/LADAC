% This test loads default simple analytic airfoil parameters and visualizes
% the lift, drag and pitching moment coefficients vs. angle of attack

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% define angle of attack
alpha_deg = -10:0.1:10;

% create simple analytic airfoil struct
airfoil = airfoilAnalyticSimpleCreate('airfoilAnalyticSimple_params_default');

% compute aerodynamic coefficients for all angles of attack
c_L = airfoilAnalyticSimpleCl(airfoil,deg2rad(alpha_deg));
c_D = airfoilAnalyticSimpleCd(airfoil,deg2rad(alpha_deg));
c_m = airfoilAnalyticSimpleCm(airfoil,c_L);


%% plot

figure
subplot(2,2,1)
plot(alpha_deg,c_L)
grid on
xlabel('Angle of attack in deg')
ylabel('Lift coefficient')

subplot(2,2,2)
plot(alpha_deg,c_D)
grid on
xlabel('Angle of attack in deg')
ylabel('Drag coefficient')

subplot(2,2,3)
plot(alpha_deg,c_m)
grid on
xlabel('Angle of attack in deg')
ylabel('Pitching moment coefficient')

