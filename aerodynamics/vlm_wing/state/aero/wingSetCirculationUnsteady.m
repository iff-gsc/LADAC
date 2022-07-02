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

% compute unit vector in direction of freestream ([5],
% nomenclature, above eq. 12)
if wing.config.is_unsteady
    v_inf = - wing.state.aero.local_inflow.V_25 ./ repmat( vecnorm(wing.state.aero.local_inflow.V_25,2), size(wing.state.aero.local_inflow.V_25,1), 1 );
else
    v_inf = - wing.state.aero.local_inflow.V_75 ./ repmat( vecnorm(wing.state.aero.local_inflow.V_75,2), size(wing.state.aero.local_inflow.V_75,1), 1 );
end
% rotation axis for normal vector to adjust the angle of attack / incidence
wing.state.aero.circulation.rot_axis = crossFast( -v_inf, u_n );

% influence coefficients
A = zeros( wing.n_panel, wing.n_panel );

% sweep angle relative the the inflow
spanwise_vector = wingGetDimLessSpanwiseLengthVector(wing.state.geometry.vortex);
wing.interim_results.sweep = pi/2 - acosReal( abs( dot( -spanwise_vector, -v_inf, 1 ) ) ./ ( vecnorm(-v_inf,2,1) .* vecnorm(spanwise_vector,2,1) ) );
span = sum(wingGetSegmentSpan(wing.state.geometry.vortex));

% normal components of free-stream flow [4], eq. (12.8)
v_ni = - dot( v_inf, u_n );
% angle of attack of free-stream
alpha_inf = acosReal(v_ni) - pi/2;

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
    if wing.config.is_unsteady
        abs_V_i(:) = vecnorm( wing.state.aero.circulation.v_i .* wing.state.aero.local_inflow.V_25, 2 );
    else
        abs_V_i(:) = vecnorm( wing.state.aero.circulation.v_i .* wing.state.aero.local_inflow.V_75, 2 );
    end
    % Reynolds number and Mach number
    wing.state.aero.circulation.Re	= reynoldsNumber( wing.state.external.atmosphere.rho, abs_V_i, wing.state.geometry.ctrl_pt.c, wing.state.external.atmosphere.mu );
    wing.state.aero.circulation.Ma	= abs_V_i / wing.state.external.atmosphere.a .* cos(wing.interim_results.sweep); 
    % limit Mach number (to do: hard coded)
	wing.state.aero.circulation.Ma(wing.state.aero.circulation.Ma>0.73)=0.73;
    
    % quasi-steady flap deflection with sweep compensation
    [F_10,F_11] = airfoilFlapEffectiveness(wing.geometry.segments.flap_depth);
    wing.state.aero.circulation.delta_qs = airfoilFlapDeltaQs( F_10, F_11,...
        abs_V_i, wing.state.geometry.ctrl_pt.c, deg2rad(wing.state.actuators.segments.pos(1,:)), ...
        deg2rad(wing.state.actuators.segments.rate(1,:)) );
    
    if ~wing.config.is_unsteady
        % static lift coefficient
        switch wing.config.airfoil_method
            case 'analytic'
                % clean airfoil lift
                fcl = airfoilAnalytic0515Ma( wing.airfoil.analytic.wcl, wing.state.aero.circulation.Ma );
                if wing.config.is_stall
                    c_L_visc(:) = airfoilAnalytic0515AlCl( fcl, [ rad2deg(wing.state.aero.circulation.alpha_eff); wing.state.aero.circulation.Ma ] );
                else
                    % get points on lift curve
                    [c_L_alpha_max,alpha_0] = airfoilAnalytic0515ClAlphaMax( fcl, wing.state.aero.circulation.Ma );
                    % effective angle of attack for an equivalent uncambered airfoil
                    alpha_inf_0 = wing.state.aero.circulation.alpha_eff - deg2rad(alpha_0);
                    c_L_visc(:) = rad2deg(c_L_alpha_max) .* alpha_inf_0;
                end
            case 'simple'
                % clean airfoil + flap lift
                c_L_visc(:) = airfoilAnalyticSimpleCl( wing.airfoil.simple, ...
                    wing.state.aero.circulation.alpha_eff, wing.state.aero.circulation.Ma );
        end
        % flap lift
        wing.state.aero.circulation.c_L_flap(:) = 2*pi./sqrtReal(1-powerFast(wing.state.aero.circulation.Ma,2))...
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
    else
        % unsteady airfoil lift coefficient
        wing = wingSetUnstAeroState( wing );
        c_L_visc = wing.state.aero.unsteady.c_L_c;
    end
    
    % influence coefficients matrix [4], eq. (12.7)
    A(:) = dot( wing.interim_results.dimless_induced_vel_beta, repmat(u_n,1,1,wing.n_panel), 1 );
    
    if ~wing.config.is_unsteady
        % adjust the normal vector (if Delta_alpha ~= 0)
        u_n_VLM = axisAngle( u_n, wing.state.aero.circulation.rot_axis, wing.state.aero.circulation.Delta_alpha );
        % normal airspeed component for VLM
        v_ni_VLM = - dot( v_inf, u_n_VLM, 1 );
        % (dimensionless) circulation of VLM
        wing.state.aero.circulation.gamma(:) = (A \ v_ni_VLM');
        % lift coefficient of VLM
        c_L_VLM = 2*wing.state.aero.circulation.gamma * span./wing.state.geometry.ctrl_pt.c;
    else
        % VLM backwards (c_L_visc won't change)
        c_L_VLM = c_L_visc;
        wing.state.aero.circulation.gamma = 0.5*c_L_VLM / span .* wing.state.geometry.ctrl_pt.c;
        v_ni_VLM = (A * wing.state.aero.circulation.gamma(:))';
        wing.state.aero.circulation.Delta_alpha = acosReal(v_ni_VLM) - pi/2 - alpha_inf;
    end
    
    % VLM lift without any downwash
    c_L_inf = 2*pi*(alpha_inf+wing.state.aero.circulation.Delta_alpha);
    % induced angle of attack from VLM
    wing.state.aero.circulation.alpha_ind = (c_L_inf-c_L_VLM)./(2*pi);
    
    % maximum error from all panels:
    err = max( abs(c_L_VLM-c_L_visc) );
    
    % If the error is below a certain tolerance err_max, take the
    % lift coefficient as the converged value for each panel; otherwise,
    % repeat.
    if err/max(abs(c_L_visc)) < err_rel_max || err < err_abs_max
        converged = true;
    else
        wing.state.aero.circulation.num_iter = wing.state.aero.circulation.num_iter + 1;
    end
end

wing.state.aero.circulation.v_i_unit = wing.state.aero.circulation.v_i ...
    ./repmat(vecnorm(wing.state.aero.circulation.v_i,2,1),3,1);
wing.state.aero.circulation.Gamma(:) = wing.state.aero.circulation.gamma ...
    .* abs_V_i * span;
wing.state.aero.circulation.c_L(:) = c_L_visc;

end
