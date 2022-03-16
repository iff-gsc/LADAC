%% Verifying the nonlinear vortex step method with fundamental solutions
% 
% Compare the induced angle of attack of the NLVSM with the
% VLM results (VLM has only one panel in chord direction) for swept wings
% 
% The results should be very similar.
% 
% Literature:
%   [1] Schlichting, H.; Truckenbrodt, E.; "Aerodynamik des FLugzeugs -
%       Zweiter Band: Aerodynamik des Tragfl�gels (Teil II), des Rumpfes,
%       der Fl�gel-Rumpf-Anordnung und der Leitwerke", 3. Auflage,
%       Springer-Verlag, Berlin, Heidelberg, 2001
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

%% Set parameters

% set sweep and aspect ratio
sweep = deg2rad(45);
AR = 10;
% number of panels
n_panel = 50;

%% define current wing state

% define current rigid body state
alpha = deg2rad(2.1);
beta = 0;
V = 1;
h = 0;
omega = [0;0;0;];

% define actuator states
actuators_pos = [0,0];
actuators_rate = [0,0];

%% Start computation 

% create wing
wing = wingCreate( wing_parametric(AR,1,sweep,0), 50 );

% compute aerodynamics
wing = wingSetState( wing, alpha, beta, V, omega, actuators_pos, actuators_rate, [0;0;0] );

% compute influence coefficients
v_test_mat = wingGetDimlessIndVel( repmat([1;0;0],1,n_panel),wing.geometry);
v_test = -squeeze( v_test_mat(3,:,:) ) / sum( wingGetSegmentSpan( wing.geometry.vortex ) );

V_Ab = repmat( dcmBaFromAeroAngles(alpha,0)*[1;0;0],1,n_panel );
w_Ab = V_Ab(3,:);

infl_coeff = wingGetDimlessIndVel( -wing.state.aero.local_inflow.V, wing.state.geometry );
Gamma = squeeze(infl_coeff(3,:,:)) \ w_Ab';
l = wingGetSpatialVectorAlongBoundSegment( wing.geometry.vortex );

alpha_eff = alpha + inducedAlpha( 2*pi, -V_Ab, zeros(size(V_Ab)), Gamma', l, wing.geometry.ctrl_pt.c );


%% plot results
figure
plot(wing.geometry.ctrl_pt.pos(2,:), alpha*cos(wing.params.lambda)*ones(1,n_panel))
hold on
plot(wing.geometry.ctrl_pt.pos(2,:), alpha_eff)
plot(wing.geometry.ctrl_pt.pos(2,:), wing.state.aero.circulation.alpha_eff,'--')
ylim([0 inf])
grid on
xlabel('span coordinate in m')
ylabel('\alpha_{eff} in rad')
legend('infinite span','finite span (VLM)','finite span (NLVSM)','location','south')


%% functions
function alpha_i = inducedAlpha( C_L_alpha, V_Ab_inf, V_i, Gamma, l, c )

[alpha_inf,~] = aeroAngles(-V_Ab_inf);

c_L_inf = 2*pi*alpha_inf;
span = sum(l(2,:));
c_F = 2*repmat(Gamma./vecnorm(V_Ab_inf,2).^2,3,1).*cross(V_Ab_inf+V_i,span*l)./repmat((vecnorm(l(2:3,:),2).*c),3,1);
c_L = -c_F(3,:);

alpha_i = (c_L-c_L_inf)./C_L_alpha;

end

