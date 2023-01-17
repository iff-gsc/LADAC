%% Visualize elliptical wing

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% define basic wing parameters
aspect_ratio = 10;
taper_ratio = 1;
sweep_angle = 0.1;
twist_angle = 0;

% create wing
wing = wingCreate(wing_parametric(aspect_ratio,taper_ratio,sweep_angle,twist_angle),20);

prm = wing.params;

% define dimensionless spanwise coordinate eta, in -
eta_wing = 0:0.01:1;

% specific local chord for an elliptical wing, in -
local_chord = sqrt(1-eta_wing.^2);
% position of quarter chord line in relation to coordinate origin
quarter_chord = - tan(prm.lambda)*eta_wing;

% x-axis = quarter chord line
leading_edge = quarter_chord*prm.b/2 + 0.25*local_chord*prm.c(1);
trailing_edge = quarter_chord*prm.b/2 - 0.75*local_chord*prm.c(1);

% plot results
figure;
% plot(eta_wing,quarter_chord,'k')
% hold on 
% grid on 
% plot(eta_wing,leading_edge,'r')
% plot(eta_wing,trailing_edge,'b')
plot(eta_wing*prm.b/2,quarter_chord*prm.b/2,'k')
hold on 
grid on 
plot(eta_wing*prm.b/2,leading_edge,'r')
plot(eta_wing*prm.b/2,trailing_edge,'b')
axis equal
title(['Elliptical wing, b = ',num2str(prm.b),'m, AR = ',num2str(prm.AR),', \phi = ',num2str(prm.lambda*180/pi),'�'])
xlabel('s, in m')
ylabel('l, in m')
legend('1/4 chord','leading edge','trailing edge','Location','best')