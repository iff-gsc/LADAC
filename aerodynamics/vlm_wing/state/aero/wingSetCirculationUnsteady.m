function wing = wingSetCirculationUnsteady( wing ) %#codegen
% wingSetCirculationUnsteady sets the wing.state.aero.circulation struct in
% a wing struct.
%   The circulation parameters for a wing with spanwise discretization are
%   computed. Therefore, unsteady airfoil aerodynamics can be considered.
% 
% Inputs:
% 	wing           	wing struct, see wingCreate
% 
% Outputs:
% 	wing           	wing struct, see wingCreate
% 
% Literature:
%   [1] Barnes, J. P. (1997). Semi-empirical vortex step method for the
%       lift and induced drag loading of 2D and 3D wings. SAE Paper 975559.
%   [2] Van Dam, C. P. (2002). The aerodynamic design of multi-element
%       high-lift systems for transport airplanes. Progress in Aerospace
%       Sciences, 38(2), 101-144.
%   [3] Goitia, H., & Llamas, R. (2019). Nonlinear vortex lattice method
%       for stall prediction. In MATEC Web of Conferences, 304, 02006.
%   [4] Katz, J. & Plotkin, A. (2001). Low-Speed Aerodynamics. 2nd ed.
%       Cambridge University Press.
%   [5] Phillips, W. F., & Snyder, D. O. (2000). Modern adaption of
%       Prandtl's classic lifting-line theory. Jounal of Aircraft, 37(4),
%       662-670.

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

n_panel_x = 1;
n_trail = 1;

% init lift coefficient
c_L_visc    = zeros( 1, wing.n_panel, n_panel_x );
c_L_VLM     = zeros( 1, wing.n_panel, n_panel_x );

abs_V_i = zeros( 1, wing.n_panel );

% compute u_n ([5], nomenclatrue, Fig. 3) in aircraft frame
u_n = wingGetNormalVectorFromGeometry( wing.state.geometry );
u_n_VLM = zeros( 3, wing.n_panel, n_panel_x );

% compute unit vector in direction of freestream ([5],
% nomenclature, above eq. 12)
v_inf_25 = zeros( 3, wing.n_panel, n_panel_x );
v_inf_75 = zeros( 3, wing.n_panel, n_panel_x );
for i=1:3
    v_inf_25(i,:,:) = - wing.state.aero.local_inflow.V_25(i,:,:) ./ vecnorm(wing.state.aero.local_inflow.V_25,2);
    v_inf_75(i,:,:) = - wing.state.aero.local_inflow.V_75(i,:,:) ./ vecnorm(wing.state.aero.local_inflow.V_75,2);
end
% normal components of free-stream flow [4], eq. (12.8)
v_ni_25 = - dot( v_inf_25, u_n );
v_ni_75 = - dot( v_inf_75, u_n );
% angle of attack of free-stream
alpha_inf_25 = acosReal(v_ni_25) - pi/2;
alpha_inf_75 = acosReal(v_ni_75) - pi/2;
if wing.config.is_unsteady
    v_inf = v_inf_25;
    alpha_inf = alpha_inf_25;
else
    v_inf = v_inf_75;
    alpha_inf = alpha_inf_75;
end

wing.state.aero.circulation.alpha_inf(:) = alpha_inf;
% dimensionless pitch rate [1], Nomenclature and Eq. between (18) and (19)
wing.state.aero.circulation.q(:) = 2 * (alpha_inf_75 - alpha_inf_25);

% rotation axis for normal vector to adjust the angle of attack / incidence
wing.state.aero.circulation.rot_axis(:) = crossFast( -v_inf(:,:), u_n(:,:) );


cla0 = 2*pi;

span = sum(wingGetSegmentSpan(wing.state.geometry.line_25));

% ** use overworked version of algorithm presented in [3], page 3 **

if ~wing.config.is_circulation_iteration
    num_iter_max = 1;
else
    % usually the solution should converge after less than 10 iterations, large
    % actuator deflections can increase the number of iterations
    num_iter_max    = 200;
end
converged       = false;
wing.state.aero.circulation.num_iter = 0;
err_abs_max    	= 1e-7;
err_rel_max     = 1e-5;
% iterate until VLM and 2D airfoil lift coefficient are (almost) equal
while ~converged && wing.state.aero.circulation.num_iter < num_iter_max
    
    % effective angles of attack of each strip
    alpha_eff_unswept = alpha_inf - wing.state.aero.circulation.alpha_ind;
    wing.state.aero.circulation.alpha_eff = alpha_eff_unswept;
    
    % estimate induced velocity (own formula, no appropriate formula found in literature)
    w_ind = -tan(wing.state.aero.circulation.alpha_ind);
    
    % local airspeed vector normalized by local free-stream velocity in
    % direction of local stream (to do: probably u_n should be rotated
    % about alpha_inf because the downwash is probably perpendicular to
    % V_inf (aerodynamic axis system); but the difference should be very
    % small for small angles of attack)
    for i = 1:3
        wing.state.aero.circulation.v_i(i,:,:) = v_inf(i,:,:) + u_n(i,:,:) .* w_ind;
    end
    
    V_i = wing.state.aero.circulation.v_i;
    if wing.config.is_unsteady
        V_norm = vecnorm(wing.state.aero.local_inflow.V_25,2,1);
    else
        V_norm = vecnorm(wing.state.aero.local_inflow.V_75,2,1);
    end
    for i = 1:3
        V_i(i,:) = V_i(i,:) .* V_norm;
    end
    % absolute local airspeed vector
    abs_V_i(:) = mean( vecnorm( V_i, 2 ), 3 );
    
    % Reynolds number and Mach number
    V_A_i = vecnorm( wing.state.aero.local_inflow.V_75, 2, 1 );
    Ma_unswept= V_A_i / wing.state.external.atmosphere.a;
    wing.state.aero.circulation.Ma = Ma_unswept .* cos(wing.interim_results.sweep50);
    Ma_flap = Ma_unswept .* cos(wing.interim_results.sweep_flap);
    % limit Mach number (to do: hard coded)
    wing.state.aero.circulation.Ma(wing.state.aero.circulation.Ma>0.7459)=0.7459;
    beta_Ma = 1./sqrtReal(1-wing.state.aero.circulation.Ma.^2);
    beta = diag(beta_Ma);
    beta_Ma_flap = 1./sqrtReal(1-Ma_flap.^2);

    % quasi-steady flap deflection with sweep compensation
    [F_10,F_11] = airfoilFlapEffectiveness(wing.geometry.segments.flap_depth);
    wing.state.aero.circulation.delta_qs = airfoilFlapDeltaQs( F_10, F_11,...
        abs_V_i, sum(wing.state.geometry.ctrl_pt.c,3), deg2rad(wing.state.actuators.segments.pos(1,:)), ...
        deg2rad(wing.state.actuators.segments.rate(1,:)) ) .* cos(wing.geometry.segments.flap_sweep);
    if ~wing.config.is_unsteady
        % static lift coefficient
        switch wing.config.airfoil_method
            case 'analytic'
                % clean airfoil lift
                fcl = airfoilAnalytic0515Ma( wing.airfoil.analytic.wcl, wing.state.aero.circulation.Ma );
                wing.state.aero.circulation.cla = rad2deg(fcl(2,:));
                
                % get points on lift curve
                [~,alpha_0] = airfoilAnalytic0515ClAlphaMax( fcl, wing.state.aero.circulation.Ma );
                
                wing.state.aero.circulation.alpha_eff = (alpha_eff_unswept-deg2rad(alpha_0))...
                    ./cos(wing.interim_results.sweep) + deg2rad(alpha_0);
                
                if wing.config.is_stall
                    for i = 1:n_panel_x
                        c_L_visc(:,:,i) = airfoilAnalytic0515AlCl( fcl, [ rad2deg(wing.state.aero.circulation.alpha_eff(:,:,i)); wing.state.aero.circulation.Ma ] );
                    end
                else
                    % get points on lift curve
                    [c_L_alpha_max,alpha_0] = airfoilAnalytic0515ClAlphaMax( fcl, wing.state.aero.circulation.Ma );
                    % effective angle of attack for an equivalent uncambered airfoil
                    alpha_inf_0 = wing.state.aero.circulation.alpha_eff - deg2rad(alpha_0);
                    c_L_visc(:) = rad2deg(c_L_alpha_max) .* alpha_inf_0;
                end
            case 'simple'
                % clean airfoil + flap lift
                wing.state.aero.circulation.alpha_eff = (alpha_eff_unswept-deg2rad(wing.airfoil.simple.alpha_0))...
                    ./cos(wing.interim_results.sweep) + deg2rad(wing.airfoil.simple.alpha_0);
                for i = 1:n_panel_x
                    c_L_visc(:,:,i) = airfoilAnalyticSimpleCl( wing.airfoil.simple, ...
                        wing.state.aero.circulation.alpha_eff(:,:,i), wing.state.aero.circulation.Ma );
                    wing.state.aero.circulation.cla(:) = wing.airfoil.simple.c_L_alpha;
                end
        end
        % flap lift
        wing.state.aero.circulation.c_L_flap(:) = 2*pi.*beta_Ma_flap ...
            .*wing.state.aero.circulation.delta_qs;
        % 2nd actuator lift
        c_L_act2 = zeros(size(c_L_visc));
        switch wing.config.actuator_2_type
            case 'none'
                % do nothing
            case 'micro-tab'
                % 2nd actuator lift
                [ c_L_act2(:), ~, ~ ] = airfoilMicroTabDeltaCoeff( ...
                    wing.airfoil.micro_tab, wing.state.aero.circulation, ...
                    wing.state.actuators.segments.pos(2,:) );
        end
        % total lift
        c_L_visc = c_L_visc ...
            + wing.state.aero.circulation.c_L_flap + c_L_act2;
        % adjust the incidence angle of the VLM
        c_L_visc = c_L_visc .* cos(wing.interim_results.sweep);
        wing.state.aero.circulation.Delta_alpha = c_L_visc ./ (cla0) - alpha_eff_unswept;
    else
        % unsteady airfoil lift coefficient
        wing = wingSetUnstAeroState( wing );
        c_L_visc = wing.state.aero.unsteady.c_L_c;
        wing.state.aero.circulation.Delta_alpha = c_L_visc ./ (cla0) - alpha_eff_unswept;
    end
    
    if wing.config.is_circulation_iteration
        % adjust the normal vector (if Delta_alpha ~= 0)
        u_n_VLM(:) = axisAngle( u_n(:,:), wing.state.aero.circulation.rot_axis(:,:), wing.state.aero.circulation.Delta_alpha(:,:) );
        % normal airspeed component for VLM
        v_ni_VLM = - dot( v_inf, u_n_VLM, 1 );
        if wing.config.is_unsteady
            v_ni_wake = (wing.interim_results.AIC_t * wing.state.aero.circulation.gamma_trail(:) )';
            v_ni_VLM = v_ni_VLM - v_ni_wake;
        end
        if n_trail == 1
            wing.state.aero.circulation.gamma_trail(:) = wing.state.aero.circulation.gamma;
        end
        v_ni_wake = wing.interim_results.AIC_t * wing.state.aero.circulation.gamma_trail(:);
        wing.state.aero.circulation.gamma(:) = wing.interim_results.AIC_b \ (v_ni_VLM(:)-v_ni_wake(:));
        wing.state.aero.circulation.gamma(:,:,2:end) = wing.state.aero.circulation.gamma(:,:,2:end) - wing.state.aero.circulation.gamma(:,:,1:end-1);
        % lift coefficient of VLM
        c_L_VLM(:) = 2*wing.state.aero.circulation.gamma * span./wing.state.geometry.ctrl_pt.c;
    else
        % VLM backwards (c_L_visc won't change)
        c_L_VLM(:) = c_L_visc;
        wing.state.aero.circulation.gamma = 0.5*c_L_VLM / span .* wing.state.geometry.ctrl_pt.c;
        v_ni_wake = (wing.interim_results.AIC_t * wing.state.aero.circulation.gamma(:) )';
        v_ni_VLM = (wing.interim_results.AIC_b * wing.state.aero.circulation.gamma(:))' + v_ni_wake;
        alpha_eff_VLM = -asinReal(v_ni_VLM);
        wing.state.aero.circulation.Delta_alpha = alpha_eff_VLM - alpha_inf;
    end
    
    % maximum error from all panels:
    err = max( abs( mean(c_L_VLM-c_L_visc, 3 ) ) );
    
    err_rel = err/max(abs(c_L_visc(:)));
    
    % VLM lift without any downwash
    c_L_inf = cla0.*(alpha_inf+wing.state.aero.circulation.Delta_alpha);
    
    % induced angle of attack from VLM (add successive increments to avoid
    % instability)
    if wing.config.is_unsteady
        alpha_ind = (c_L_inf-c_L_VLM)./(cla0);
    else
        alpha_ind = (c_L_inf-c_L_VLM)./(cla0);
    end
    alpha_ind_dt = 0.5 * (alpha_ind - wing.state.aero.circulation.alpha_ind);
    wing.state.aero.circulation.alpha_ind = wing.state.aero.circulation.alpha_ind + alpha_ind_dt*1;
    
    % If the error is below a certain tolerance err_max, take the
    % lift coefficient as the converged value for each panel; otherwise,
    % repeat.
    if  (err_rel < err_rel_max || err < err_abs_max)
        converged = true;
    else
        wing.state.aero.circulation.num_iter = wing.state.aero.circulation.num_iter + 1;
    end
end

wing.state.aero.circulation.v_i_unit = wing.state.aero.circulation.v_i ...
    ./repmat(vecnorm(wing.state.aero.circulation.v_i,2,1),3,1);
wing.state.aero.circulation.Gamma(:) = sum(wing.state.aero.circulation.gamma,3) ...
    .* abs_V_i * span;
wing.state.aero.circulation.c_L(:) = c_L_VLM;
wing.state.aero.circulation.alpha_inf(:) = alpha_inf;

end
