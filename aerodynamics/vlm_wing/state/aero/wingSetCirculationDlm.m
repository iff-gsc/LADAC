function wing = wingSetCirculationDlm( wing ) %#codegen
% wingSetCirculationDlm sets the wing.state.aero.circulation struct in
% a wing struct.
%   The circulation parameters for a wing with spanwise discretization are
%   computed. Therefore, unsteady airfoil aerodynamics can be considered.
% 
% Inputs:
% 	wing           	wing struct, see wingCreate
% 
% Outputs:
% 	wing           	wing struct, see wingCreate

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************


abs_V_i = zeros( 1, wing.n_panel );

% compute u_n ([5], nomenclatrue, Fig. 3) in aircraft frame
u_n = wingGetNormalVectorFromGeometry( wing.state.geometry );

% [katz & Plotkin, tau_i in eq. (13.149)]
tau_i = diff(wing.geometry.vortex.pos(:,1:end-1,:)+wing.state.geometry.vortex.pos(:,2:end,:),1,3);
% tau_j must be perpendicular to u_n and tau_i ??
tau_i_length = vecnorm(tau_i,2,1);
for i=1:3
    tau_i(i,:,:) = tau_i(i,:,:) ./ tau_i_length;
end

tau_j = cross( tau_i, u_n );

u_n_VLM = zeros( 3, wing.n_panel, wing.n_panel_x );

% compute unit vector in direction of freestream ([5],
% nomenclature, above eq. 12)
v_inf = zeros( 3, wing.n_panel, wing.n_panel_x );
for i=1:3
    v_inf(i,:,:) = - wing.state.aero.local_inflow.V_75(i,:,:) ./ vecnorm(wing.state.aero.local_inflow.V_75,2);
end

v_wake = zeros( 3, wing.n_panel, wing.n_panel_x );

% influence coefficients
Ab = zeros( wing.n_panel*wing.n_panel_x, wing.n_panel*wing.n_panel_x );
At = zeros( wing.n_panel*wing.n_panel_x, wing.n_panel*wing.n_trail );

% sweep angle relative the the inflow
spanwise_vector = wingGetDimLessSpanwiseLengthVector(wing.state.geometry.line_25);
wing.interim_results.sweep = pi/2 - acosReal( abs( dot( -spanwise_vector, -v_inf(:,:,1), 1 ) ) ./ ( vecnorm(-v_inf(:,:,1),2,1) .* vecnorm(spanwise_vector,2,1) ) );
span = sum(wingGetSegmentSpan(wing.state.geometry.line_25));
    
% absolute local airspeed vector
if wing.config.is_unsteady
    abs_V_i(:) = mean( vecnorm( v_inf .* wing.state.aero.local_inflow.V_25, 2 ), 3 );
else
    abs_V_i(:) = mean( vecnorm( v_inf .* wing.state.aero.local_inflow.V_75, 2 ), 3 );
end
% Mach number
wing.state.aero.circulation.Ma	= abs_V_i / wing.state.external.atmosphere.a .* cos(wing.interim_results.sweep); 
% limit Mach number (to do: hard coded)
wing.state.aero.circulation.Ma(wing.state.aero.circulation.Ma>0.9)=0.9;

beta_Ma = 1./sqrtReal(1-wing.state.aero.circulation.Ma.^2);

% % quasi-steady flap deflection with sweep compensation
% [F_10,F_11] = airfoilFlapEffectiveness(wing.geometry.segments.flap_depth);
% wing.state.aero.circulation.delta_qs = airfoilFlapDeltaQs( F_10, F_11,...
%     abs_V_i, sum(wing.state.geometry.ctrl_pt.c,3), deg2rad(wing.state.actuators.segments.pos(1,:)), ...
%     deg2rad(wing.state.actuators.segments.rate(1,:)) );
% 
% % flap lift
% wing.state.aero.circulation.c_L_flap(:) = 2*pi./sqrtReal(1-powerFast(wing.state.aero.circulation.Ma,2))...
%     .*wing.state.aero.circulation.delta_qs;
% % 2nd actuator lift
% c_L_act2 = zeros(size(c_L_visc_25));
% switch wing.config.actuator_2_type
%     case 'none'
%         % do nothing
%     case 'micro-tab'
%         % 2nd actuator lift
%         [ c_L_act2(:), ~, ~ ] = airfoilMicroTabDeltaCoeff( ...
%             wing.airfoil.micro_tab, wing.state.aero.circulation, ...
%             wing.state.actuators.segments.pos(2,:) );
% end
% % total lift
% c_L_visc_25 = mean(c_L_visc,3) ...
%     + wing.state.aero.circulation.c_L_flap + c_L_act2;



% influence coefficients matrix [4], eq. (12.7)
for i = 1:wing.n_panel*wing.n_panel_x
    Ab(:,i) = dot( wing.interim_results.AIC_b_beta(:,:,i), u_n(:,:), 1 );
end
for j = 1:wing.n_panel*wing.n_trail
    At(:,j) = dot( wing.interim_results.AIC_t(:,:,j), u_n(:,:), 1 );
    v_wake(:,:) = v_wake(:,:) + ...
        wing.interim_results.AIC_t(:,:,j) * wing.state.aero.circulation.gamma_trail(j);
end
v_inf_plus_wake = v_inf + v_wake;

% to do
u_n_VLM(:) = u_n(:);
% normal airspeed component for VLM
v_ni_VLM = - dot( v_inf, u_n_VLM, 1 );
if wing.config.is_unsteady
    v_ni_VLM(:) = v_ni_VLM(:) - At * wing.state.aero.circulation.gamma_trail(:);
else
    Ab(:,end-wing.n_panel*wing.n_trail+1:end) = Ab(:,end-wing.n_panel*wing.n_trail+1:end) + At;
end
% (dimensionless) circulation of VLM
wing.state.aero.circulation.gamma(:) = (Ab \ v_ni_VLM(:));

Delta_gamma_i = cat( 3, wing.state.aero.circulation.gamma(:,:,1), ...
    wing.state.aero.circulation.gamma(:,:,2:end) - wing.state.aero.circulation.gamma(:,:,1:end-1) );
Delta_gamma_j = cat( 2, wing.state.aero.circulation.gamma(:,1,:), ...
    wing.state.aero.circulation.gamma(:,2:end,:) - wing.state.aero.circulation.gamma(:,1:end-1,:) );
% Delta_gamma_j = cat( 2, wing.state.aero.circulation.gamma(:,1,:), ...
%     wing.state.aero.circulation.gamma(:,2:end/2,:) - wing.state.aero.circulation.gamma(:,1:end/2-1,:), ...
%     wing.state.aero.circulation.gamma(:,end-1:-1:end/2+1,:) - wing.state.aero.circulation.gamma(:,end:-1:end/2+2,:), ...
%     wing.state.aero.circulation.gamma(:,end,:) );
Delta_gamma_last = cat( 3, wing.state.aero.circulation.gamma_last(:,:,1), ...
    wing.state.aero.circulation.gamma_last(:,:,2:end) - wing.state.aero.circulation.gamma_last(:,:,1:end-1) );
% lift coefficient of VLM
Delta_b = wingGetSegmentSpan(wing.state.geometry.vortex);
wing.state.aero.unsteady.c_L_c = ( ...
    2 * Delta_gamma_i * span./wing.state.geometry.ctrl_pt.c .* dot(v_inf_plus_wake,tau_i) ...
    - 2 * Delta_gamma_j * span./Delta_b(:,:,1:end-1) .* dot(v_inf_plus_wake,tau_j) ...
    ) .* beta_Ma;

dt = wing.interim_results.wake.dx / wing.state.body.V;
wing.state.aero.unsteady.c_L_nc = 2 * (Delta_gamma_i-Delta_gamma_last)/dt * span / ...
    ( wing.state.external.atmosphere.rho * wing.state.body.V ) .* beta_Ma;


% to do
w_ind = zeros( 1, wing.n_panel, wing.n_panel_x );

for i = 1:3
    wing.state.aero.circulation.v_i(i,:,:) = v_inf(i,:,:) + u_n(i,:,:) .* w_ind;
end


wing.state.aero.circulation.v_i_unit = wing.state.aero.circulation.v_i ...
    ./repmat(vecnorm(wing.state.aero.circulation.v_i,2,1),3,1);
wing.state.aero.circulation.Gamma(:) = sum(Delta_gamma_i,3) ...
    .* abs_V_i * span;

wing.state.aero.circulation.c_L(:) = wing.state.aero.unsteady.c_L_c;


end
