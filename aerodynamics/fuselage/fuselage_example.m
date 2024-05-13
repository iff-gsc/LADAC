% test the fuselage code

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

%% create default fuselage
fuselage = fuselageCreate('fuselage_params_default',5,30);
% fuselage = fuselageCreate('fuselage_params_default',5,20,'flexible',structure_red);

% visualize fuselage
figure
fuselagePlotGeometry(fuselage);

%% create wing

wing_prm = wing_parametric(10,1,0,0);
wing_prm.b = wing_prm.b*2;
wing_prm.c = wing_prm.c*2;
wing_prm.x = -fuselage.params.total_length/2;
wing_prm.z = 0.01;
wing_prm.i = 0.0;
wing = wingCreate(wing_prm,40);

%% define flight condition

alpha = 0.1;
beta = 0;
V = 10;
omega = [ 0; 0; 0 ];
xyz_cg = [ -10; 0; 0 ];
structure_pos = fuselage.state.geometry.cntrl_pos;
structure_vel = fuselage.state.geometry_deriv.border_pos_dt(:,1:end-1) ...
    + diff(fuselage.state.geometry_deriv.border_pos_dt,1,2);

%% compute wing state

wing = wingSetState(wing,alpha,beta,V,omega,[0,0],[0,0],zeros(3,1));

%% external

h = 1000;

v_ind = wingFuselageInducedVel(  wing.geometry.line_25.pos+wing.geometry.origin, wing.state.aero.circulation.Gamma, mean(wing.params.c), fuselage.geometry.border_pos );

%% compute state

fuselage = fuselageSetState( fuselage, alpha, beta, V, omega, xyz_cg, ...
    'atmosphere', isAtmosphere(h), 'wind', ...
    [zeros( 2, fuselage.n_segments+1 ); v_ind(:)'], zeros( 3, fuselage.n_segments+1 ), ...
    'structure_pos', structure_pos, 'structure_vel', structure_vel );

%% plot
figure
% plot( -fuselage.geometry.cntrl_pos(1,:), -fuselage.state.aero.R_Ab_i(3,:)/((wing_prm.i+alpha)*fuselage.external.atmosphere.rho/2*V^2*mean(wing.params.c)) )
plot( -fuselage.geometry.cntrl_pos(1,:), -fuselage.state.aero.R_Ab_i(3,:) )
grid on
xlabel('Fuselage center line position, in m')
ylabel('Lift, in N')


