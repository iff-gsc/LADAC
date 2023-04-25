%% Verifying the nonlinear vortex step method with fundamental solutions
% 
% Compare the induced angle of attack of the NLVSM with the
% VLM results (VLM has only one panel in chord direction) for swept wings
% 
% The results should be very similar.
% 
% Literature:
%   [1] Schlichting, H.; Truckenbrodt, E.; "Aerodynamik des FLugzeugs -
%       Zweiter Band: Aerodynamik des Tragfluegels (Teil II), des Rumpfes,
%       der Fluegel-Rumpf-Anordnung und der Leitwerke", 3. Auflage,
%       Springer-Verlag, Berlin, Heidelberg, 2001
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

%% Set parameters

% set sweep and aspect ratio
sweep = deg2rad(45);
AR = 10;
twist_angle = 0;% 0.1;
% number of panels
n_panel = 40;
n_panel_x = 1;

%% define current wing state

% define current rigid body state
alpha = deg2rad(2.1);
beta = 0;
V = 25.0;
h = 0;
omega = [0;0;0;];

% define actuator states
actuators_pos = [0,0];
actuators_rate = [0,0];

%% Start computation 

% create wing
wing = wingCreate( wing_parametric(AR,1,sweep,twist_angle), n_panel, n_panel_x, 'method', 'IVLM', 1 );

% compute aerodynamics
wing = wingSetState( wing, alpha, beta, V, omega, actuators_pos, actuators_rate, [0;0;0] );

% compute influence coefficients
Ma = V*cos(wing.interim_results.sweep50);
v_test_mat = wingGetDimlessIndVel( repmat([1;0;0],1,n_panel,n_panel_x),wing.geometry,wing.interim_results.wake,Ma,false);
v_test = -squeeze( v_test_mat(3,:,:) ) / sum( wingGetSegmentSpan( wing.geometry.line_25 ) );

V_Ab = repmat( dcmBaFromAeroAngles(alpha,0)*[1;0;0], 1, n_panel*n_panel_x );
V_A_local = zeros(3,n_panel*n_panel_x);
for i=1:n_panel*n_panel_x
    V_A_local(:,i) = dcmBaFromAeroAngles(wing.geometry.ctrl_pt.local_incidence(i),0)*V_Ab(:,i);
end
w_A_local = V_A_local(3,:);

infl_coeff = wingGetDimlessIndVel( -wing.state.aero.local_inflow.V_75, wing.geometry, wing.interim_results.wake, Ma, false );
A = wing.interim_results.AIC_b(3,:,1:n_panel*n_panel_x);
A(:,:,end-n_panel+1:end) = A(:,:,end-n_panel+1:end) + wing.interim_results.AIC_t(3,:,:);
Gamma = squeeze(A) \ w_A_local';
Gamma = reshape(Gamma,1,n_panel,[]);
Gamma(1,:,2:end) = Gamma(1,:,2:end) - Gamma(1,:,1:end-1);
Gamma = Gamma/n_panel_x;
l = wingGetSpatialVectorAlongBoundSegment( wing.geometry.vortex );
l = l(:,:,1:end-1);

c_L_alpha = 2*pi;

alpha_eff = alpha + wing.geometry.ctrl_pt.local_incidence(:)' + inducedAlpha( c_L_alpha, -V_Ab, zeros(size(V_Ab)), Gamma(:)', l(:,:), wing.geometry.ctrl_pt.c(:)', wing.geometry.ctrl_pt.local_incidence(:)' );
alpha_eff = reshape(alpha_eff,1,n_panel,[]);

%% plot results
figure
plot(wing.geometry.ctrl_pt.pos(2,:,1), (alpha+ wing.geometry.ctrl_pt.local_incidence(:,:,1)).*cos(wing.interim_results.sweep))
hold on
plot(wing.geometry.ctrl_pt.pos(2,:,1), mean(alpha_eff,3)./cos(wing.interim_results.sweep50))
plot(wing.geometry.ctrl_pt.pos(2,:,1), mean(wing.state.aero.circulation.alpha_eff,3),'--')
ylim([0 inf])
grid on
xlabel('span coordinate in m')
ylabel('\alpha_{eff} in rad')
legend('infinite span','finite span (VLM)','finite span (NLVSM)','location','south')


%% functions
function alpha_i = inducedAlpha( c_L_alpha, V_Ab_inf, V_i, Gamma, l, c, local_incidence )

[alpha_inf,~] = aeroAngles(-V_Ab_inf);

alpha_inf = alpha_inf + local_incidence;

c_L_inf = 2*pi*alpha_inf;
span = sum(l(2,:));
c_F = 2*repmat(Gamma./vecnorm(V_Ab_inf,2).^2,3,1).*cross(V_Ab_inf+V_i,span*l)./repmat((vecnorm(l(2:3,:),2).*c),3,1);
c_L = -c_F(3,:);

alpha_i = (c_L-c_L_inf)./c_L_alpha;

end

