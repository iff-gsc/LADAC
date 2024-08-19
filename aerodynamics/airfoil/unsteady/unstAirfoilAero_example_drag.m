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
Ma = 0.4;
% reduced frequency, -
k = 0.0378;
% chord, m
c = 2.8;
% speed of sound, m/s
a = 300;
% mean angle of attack
alpha_0 = 0;
% angle of attack oscillation amplitude
Delta_alpha = 8;

% load analytic coefficient parameters for F15_bl (neural network weights)
airfoil = airfoilAnalyticSimpleCreate('airfoilAnalyticSimple_params_default');
airfoil_bus = struct2bus_(airfoil);
% airspeed, m/s
V = Ma*a;

% simulation time
t_sim = 5 * 1/(k*2*V/c/(2*pi));

%% simulate

sim('unstAirfoilAero_sim_example_drag');

%% plot

plot( rad2deg(simout.Data(round(end/2):end,1)), simout.Data(round(end/2):end,2) )
hold on
plot( rad2deg(simout.Data(round(end/2):end,1)), simout.Data(round(end/2):end,3) )
grid on
legend('static','unsteady','location','best')
xlabel('Angle of attack, deg')
ylabel('Drag coefficient, -')

xlim([-10,10])
ylim([-0.05,0.1])