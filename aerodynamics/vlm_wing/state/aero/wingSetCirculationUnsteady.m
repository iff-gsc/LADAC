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
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% init lift coefficient
c_L_visc = zeros( 1, wing.n_panel );

abs_V_i = zeros( 1, wing.n_panel );

% compute u_n ([5], nomenclatrue, Fig. 3) in aircraft frame
u_n = wingGetNormalVectorFromGeometry( wing.state.geometry );
u_n_VLM = u_n;

% compute unit vector in direction of freestream ([5],
% nomenclature, above eq. 12)
v_inf = - wing.state.aero.local_inflow.V ./ repmat( vecnorm(wing.state.aero.local_inflow.V,2), size(wing.state.aero.local_inflow.V,1), 1 );

% rotation axis for normal vector to adjust the angle of attack / incidence
rot_axis = cross( -v_inf, u_n, 1 );

% sweep angle relative the the inflow
spanwise_vector = wingGetDimLessSpanwiseLengthVector(wing.state.geometry.vortex);
wing.interim_results.sweep = pi/2 - acosReal( abs( dot( -spanwise_vector, -v_inf, 1 ) ) ./ ( vecnorm(-v_inf,2,1) .* vecnorm(spanwise_vector,2,1) ) );
span = sum(wingGetSegmentSpan(wing.state.geometry.vortex));

% normal components of free-stream flow [4], eq. (12.8)
v_ni = - dot( v_inf, u_n );
% angle of attack of free-stream
alpha_inf = - atan(v_ni);

% initial error of the lift coefficient (positive sign usually saves one
% iteration)
c_L_error_old = ones( 1, wing.n_panel );
% RPROP optimization parameters
rprop_incr = 1.2;
rprop_descr = 0.7;
% initial factor/gradient for update of alpha_ind
factor = ones( 1, wing.n_panel );

% ** use overworked version of algorithm presented in [3], page 3 **

if ~wing.config.is_circulation_iteration
    num_iter_max = 1;
else
    % usually the solution should converge after less than 10 iterations, large
    % actuator deflections can increase the number of iterations
    num_iter_max    = 100;
end
converged       = false;
wing.state.aero.circulation.num_iter = 0;
err_abs_max    	= 1e-5;
err_rel_max     = 1e-3;
% iterate until VLM and 2D airfoil lift coefficient are (almost) equal
while ~converged && wing.state.aero.circulation.num_iter < num_iter_max
    
    % effective angles of attack of each strip
    wing.state.aero.circulation.alpha_eff = alpha_inf - wing.state.aero.circulation.alpha_ind;
    % estimate induced velocity (own formula, no appropriate formula found in literature)
    w_ind = -tan(wing.state.aero.circulation.alpha_ind);

    % local airspeed vector normalized by local free-stream velocity in
    % direction of local stream (to do: probably u_n should be rotated
    % about alpha_inf because the downwash is probably perpendicular to
    % V_inf (aerodynamic axis system); but the difference should be very
    % small for small angles of attack)
    wing.state.aero.circulation.v_i = v_inf + u_n .* repmat( w_ind, 3, 1 );
    % absolute local airspeed vector
    abs_V_i(:) = vecnorm( wing.state.aero.circulation.v_i .* wing.state.aero.local_inflow.V, 2 );
    % Reynolds number and Mach number
    wing.state.aero.circulation.Re	= reynoldsNumber( wing.state.external.atmosphere.rho, abs_V_i, wing.state.geometry.ctrl_pt.c, wing.state.external.atmosphere.mu );
    wing.state.aero.circulation.Ma	= abs_V_i / wing.state.external.atmosphere.a .* cos(wing.interim_results.sweep); 
    % limit Mach number
	wing.state.aero.circulation.Ma(wing.state.aero.circulation.Ma>0.95)=0.95;
    
    % quasi-steady flap deflection with sweep compensation
    [F_10,F_11] = airfoilFlapEffectiveness(wing.geometry.segments.flap_depth);
    wing.state.aero.circulation.delta_qs = airfoilFlapDeltaQs(F_10,F_11,abs_V_i,wing.state.geometry.ctrl_pt.c,deg2rad(wing.state.actuators.segments.pos(1,:)),deg2rad(wing.state.actuators.segments.rate(1,:))) ...
        .* cos(wing.interim_results.sweep).^2;
    
    if ~wing.config.is_unsteady
        % static lift coefficient
        switch wing.config.airfoil_method
            case 'analytic'
                % clean airfoil lift
                fcl = airfoilAnalytic0515Ma( wing.airfoil.analytic.wcl, wing.state.aero.circulation.Ma, wing.airfoil.analytic.ncl, wing.airfoil.analytic.ocl );
                c_L_visc(:) = airfoilAnalytic0515AlCl( fcl, [ rad2deg(wing.state.aero.circulation.alpha_eff(:)), wing.state.aero.circulation.Ma(:) ] );
            case 'simple'
                % clean airfoil + flap lift
                c_L_visc(:) = airfoilAnalyticSimpleCl( wing.airfoil.simple, ...
                    wing.state.aero.circulation.alpha_eff, wing.state.aero.circulation.Ma );
        end
        % flap lift
        wing.state.aero.circulation.c_L_flap(:) = 2*pi./sqrtReal(1-wing.state.aero.circulation.Ma.^2)...
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
        wing.state.aero.circulation.Delta_alpha = c_L_visc / (2*pi) - wing.state.aero.circulation.alpha_eff;
        % adjust the normal vector (if Delta_alpha ~= 0)
        u_n_VLM = axisAngle( u_n, rot_axis, wing.state.aero.circulation.Delta_alpha );
    else
        % unsteady airfoil lift coefficient
        wing = wingSetUnstAeroState( wing );
        c_L_visc = wing.state.aero.unsteady.c_L_c;
        % adjust the incidence angle of the VLM
        wing.state.aero.circulation.Delta_alpha = c_L_visc / (2*pi) - wing.state.aero.unsteady.alpha_eff;
        % adjust the normal vector (if Delta_alpha ~= 0)
        u_n_VLM = axisAngle( u_n, rot_axis, wing.state.aero.circulation.Delta_alpha ...
            - ( wing.state.aero.circulation.alpha_eff - wing.state.aero.unsteady.alpha_eff ) );
    end
    
    % normal airspeed component for VLM
    v_ni_VLM = - dot( v_inf, u_n_VLM, 1 );
    % influence coefficients matrix [4], eq. (12.7)
    A = zeros( wing.n_panel, wing.n_panel );
    A(:) = dot( wing.interim_results.dimless_induced_vel_beta, repmat(u_n_VLM,1,1,wing.n_panel), 1 );
    % (dimensionless) circulation of VLM
    wing.state.aero.circulation.gamma(:) = (A \ v_ni_VLM');
    % lift coefficient of VLM
    c_L_VLM = 2*wing.state.aero.circulation.gamma * span./wing.state.geometry.ctrl_pt.c;
    
    % maximum error from all panels:
    err = max( abs(c_L_VLM-c_L_visc) );
    
    % If the error is below a certain tolerance err_max, take the
    % lift coefficient as the converged value for each panel; otherwise,
    % repeat.
    if err/max(abs(c_L_visc)) < err_rel_max || err < err_abs_max
        converged = true;
    else
        wing.state.aero.circulation.num_iter = wing.state.aero.circulation.num_iter + 1;
    
        % error between viscous lift coefficient and VLM lift coefficient
        c_L_error = c_L_visc - c_L_VLM;
        
        % Now the induced angle of attack (for the viscous lift computation)
        % must be updated so that the error decreases in the next iteration:
        % The choice of the variable "factor" is cruicial for stability and
        % performance. If the factor is chosen too high, the iteration will
        % become unstable, if the factor is chosen too low, the iteration will
        % converge very slowly.
        % A maximum factor of 1 should be chosen for stable convergence.
        % The factor should be further decreased if the viscous lift curve
        % slope is greater than 2*pi. In fact, the maximum factor should be
        % (2*pi)/c_L_alpha_viscous.
        
        % As each call of this while loop is relatively costly, an
        % automatic adaption of the factor was implemented here. The adaption
        % is very similar to the RPROP algorithm.
        % If the lift curve slope changes the sign (e.g. stall), probably
        % further modifications on this RPROP algorithm are required.
        if wing.config.is_circulation_iteration
            if wing.state.aero.circulation.num_iter > 1
                is_c_L_error_same_sign = (sign(c_L_error)==sign(c_L_error_old));
                factor(is_c_L_error_same_sign) = factor(is_c_L_error_same_sign) * rprop_incr;
                factor(~is_c_L_error_same_sign) = factor(~is_c_L_error_same_sign) * rprop_descr;
            end
            c_L_error_old = c_L_error;
        end
        
        % update induced angle of attack for viscous lift computation
        wing.state.aero.circulation.alpha_ind = wing.state.aero.circulation.alpha_ind + 1 * factor .* c_L_error ./ (2*pi);
    end
end

wing.state.aero.circulation.Gamma(:) = wing.state.aero.circulation.gamma ...
    .* abs_V_i * span;
wing.state.aero.circulation.c_L(:) = c_L_visc;

end