% Example script

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

%% Create vortex lattice method wing
% (note that the "map" option is no longer available for airfoils)
wing = wingCreate('wing_params_Arkbird_simple',30);

%% define current wing state

% define current rigid body state
alpha = deg2rad(2);
beta = deg2rad(0);
V = 20;
omega = [0;0;0];
h = 0;

% cg position in wing coordinate system
xyz_cg = zeros(3,1);

% define actuator states
actuators_pos = [ 10 0 ];
actuators_rate = [ 0 0 ];

%% compute wing state

wing = wingSetState(wing, alpha, beta, V, omega, ...
    actuators_pos, actuators_rate, xyz_cg, 'atmosphere', isAtmosphere(h), ...
    'wind', zeros( 3, wing.n_panel ), zeros( 3, wing.n_panel ) );


%% plot results

wingPlotGeometry(wing,3)

figure
y = -wing.state.aero.coeff_loc.c_XYZ_b(3,:);
plot( wing.geometry.ctrl_pt.pos(2,:)/wing.params.b*2, y );
xlabel('dimensionless span')
ylim([min(0,min(y)) inf])
ylabel('Lift coefficient')
grid on