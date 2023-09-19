function wing = wingSetUnstAeroState( wing ) %#codegen
%wingSetUnstAeroState sets the wing.state.aero.circulation.unsteady struct
% of a wing struct.
%
% Inputs:
% 	wing           	wing struct, see wingCreate
% 
% Outputs:
% 	wing           	wing struct, see wingCreate
% 
% Literature:
%   [1] Leishman, J. G., & Nguyen, K. Q. (1990). State-space representation
%       of unsteady airfoil behavior. AIAA journal, 28(5), 836-844.
%   [2] Leishman, J. (1989, April). State-space model for unsteady airfoil
%       behavior and dynamic stall. In 30th Structures, Structural Dynamic
%       and Materials Conference (p. 1319).
%   [3] Leishman, J. G., & Beddoes, T. S. (1989). A Semi?Empirical model for
%       dynamic stall. Journal of the American Helicopter society, 34(3),
%       3-17.
%   [4] Hansen, M. H., Gaunaa, M., & Madsen, H. A. (2004). A Beddoes-
%       Leishman type dynamic stall model in state-space and indicial
%       formulations.
% 
% See also: wingSetCirculationUnsteady, wingCreateState, wingCreate
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% init
alpha_inf_0 = zeros( 1, wing.n_panel );

% use raw inflow in the first step and take into account downwash
% afterwards
cos_sweep = cos(wing.interim_results.sweep);
abs_V_i = vecnorm( wing.state.aero.local_inflow.V_25, 2 ) .* cos_sweep;
c_i = wing.state.geometry.ctrl_pt.c ./ cos_sweep;

% dimensionless pitch rate [1], Nomenclature and Eq. between (18) and (19)
q = wing.state.aero.circulation.q;

% to do: explain why aerodynamic center is at 0.25c
x_ac = 0.25 * ones( 1, wing.n_panel );

alpha_eff = wing.state.aero.circulation.alpha_eff;

switch wing.config.airfoil_method
    case 'analytic'

        % get coefficients of analytic functions for different Mach numbers
        fcl = airfoilAnalytic0515Ma( wing.airfoil.analytic.wcl, wing.state.aero.circulation.Ma );
        fcd = airfoilAnalytic0515Ma( wing.airfoil.analytic.wcd, wing.state.aero.circulation.Ma );
        fcm = airfoilAnalytic0515Ma( wing.airfoil.analytic.wcm, wing.state.aero.circulation.Ma );
        
        if ~wing.config.is_stall
            x_ac(:) = airfoilAnalyticBlXac( fcm );
        end
        
        wing.state.aero.circulation.cla = rad2deg(fcl(2,:));

        % get points on lift curve
        [c_L_alpha_max,alpha_0] = airfoilAnalytic0515ClAlphaMax( fcl, wing.state.aero.circulation.Ma );
        
        alpha_eff = (alpha_eff-deg2rad(alpha_0)) ...
            ./cos(wing.interim_results.sweep) + deg2rad(alpha_0);

        % effective angle of attack for an equivalent uncambered airfoil
        alpha_inf_0 = alpha_eff - deg2rad(alpha_0);
        alpha_inf_nc = wing.state.aero.circulation.alpha_inf - deg2rad(alpha_0);
        
        % linear unsteady aerodynamics (potential flow) according to [1]
        [ wing.state.aero.unsteady.c_L_c, wing.state.aero.unsteady.c_m_c, ...
            wing.state.aero.unsteady.c_L_nc, wing.state.aero.unsteady.c_m_nc, ...
            wing.state.aero.unsteady.alpha_eff, wing.state.aero.unsteady.x_dt ...
            ] = ...
            unstAirfoilAeroFast( abs_V_i, wing.state.aero.circulation.Ma, ...
            c_i, rad2deg(c_L_alpha_max), x_ac, ...
            wing.state.aero.unsteady.x, alpha_inf_0, q, alpha_inf_nc );
        
        if wing.config.is_stall
            
            % add circulatory and non-circulatory part to obtain total
            % coefficients
            C_N_p = wing.state.aero.unsteady.c_L_c + wing.state.aero.unsteady.c_L_nc;
            c_m_p = wing.state.aero.unsteady.c_m_c + wing.state.aero.unsteady.c_m_nc;
            
            % dynamic stall according to a mix of [2], [3] and [4]
            [ wing.state.aero.unsteady.X_dt, wing.state.aero.unsteady.c_L_c(:), ...
                wing.state.aero.unsteady.c_m_c(:), wing.state.aero.unsteady.c_D(:), ...
                ~, ~, wing.state.aero.unsteady.tau_v_dt(:), ...
                wing.state.aero.unsteady.is_leading_edge_shock(:) ...
                ] = ...
                airfoilDynStall( wing.state.aero.unsteady.X, wing.state.aero.circulation.Ma, ...
                C_N_p, c_m_p, wing.state.aero.unsteady.alpha_eff, fcl, fcd, fcm, ...
                abs_V_i, c_i, alpha_eff, ...
                wing.state.aero.unsteady.tau_v, wing.config.is_le_shock );

            % avoided new variable
            wing.state.aero.unsteady.c_m_c = wing.state.aero.unsteady.c_m_c - wing.state.aero.unsteady.c_m_nc;
        else
            c_D_st = airfoilAnalytic0515AlCd( fcd, rad2deg(alpha_eff) );
            c_L_st = rad2deg(c_L_alpha_max) .* alpha_inf_0;
            wing.state.aero.unsteady.c_D(:) = ...
                unstAirfoilAeroCdNoFlutter( c_D_st, wing.state.aero.unsteady.c_L_c, alpha_eff, c_L_st );
            c_m0 = airfoilAnalyticBlCm0( fcm );
            wing.state.aero.unsteady.c_m_c(:) = wing.state.aero.unsteady.c_m_c + c_m0;
            
        end
            
    case 'simple'

        alpha_eff = (alpha_eff-wing.airfoil.simple.alpha_0) ...
            ./cos(wing.interim_results.sweep) + wing.airfoil.simple.alpha_0;
        
        % effective angle of attack for an equivalent uncambered airfoil
        alpha_inf_0(:) = alpha_eff - wing.airfoil.simple.alpha_0;
        alpha_inf_nc = wing.state.aero.circulation.alpha_inf - wing.airfoil.simple.alpha_0;

        % clean airfoil coefficients
        c_L_alpha = wing.airfoil.simple.c_L_alpha ./ ...
            sqrtReal(1-powerFast(wing.state.aero.circulation.Ma,2));
        x_ac = wing.airfoil.simple.x_ac;
        
        wing.state.aero.circulation.cla(:) = wing.airfoil.simple.c_L_alpha;
        
        [ wing.state.aero.unsteady.c_L_c, wing.state.aero.unsteady.c_m_c, ...
            wing.state.aero.unsteady.c_L_nc, wing.state.aero.unsteady.c_m_nc, ...
            wing.state.aero.unsteady.alpha_eff, wing.state.aero.unsteady.x_dt ] ...
            = unstAirfoilAeroFast( abs_V_i, wing.state.aero.circulation.Ma, ...
            c_i, c_L_alpha, x_ac, wing.state.aero.unsteady.x, ...
            alpha_inf_0, q, alpha_inf_nc );
            
        c_D_st = airfoilAnalyticSimpleCd( wing.airfoil.simple, alpha_inf_0 );
        c_L_unst = wing.state.aero.unsteady.c_L_c + wing.state.aero.unsteady.c_L_nc;
        wing.state.aero.unsteady.c_D(:) = unstAirfoilAeroCd( c_D_st, c_L_unst, ...
            alpha_inf_0, wing.state.aero.unsteady.alpha_eff );
        
end

wing.state.aero.unsteady.c_L_c = wing.state.aero.unsteady.c_L_c .* cos(wing.interim_results.sweep);

% rotate induced velocity unit vector
wing.state.aero.unsteady.v_i = axisAngle( wing.state.aero.circulation.v_i, ...
    wing.state.aero.circulation.rot_axis, alpha_inf_0 - wing.state.aero.unsteady.alpha_eff );

% flap
[ wing.state.aero.unsteady.c_L_c_flap, wing.state.aero.unsteady.z_dt(1:2,:) ] = airfoilFlapUnstLift( ...
    abs_V_i,wing.state.aero.circulation.Ma, wing.state.geometry.ctrl_pt.c, ...
    wing.state.aero.circulation.delta_qs, wing.state.aero.unsteady.z(1:2,:) );
wing.state.aero.unsteady.c_L_c = wing.state.aero.unsteady.c_L_c ...
    + wing.state.aero.unsteady.c_L_c_flap;
[ c_m_flap, wing.state.aero.unsteady.z_dt(3,:) ] = ...
    airfoilFlapUnstPitch( wing.geometry.segments.flap_depth, ...
        deg2rad(wing.state.actuators.segments.pos(1,:)), ...
        wing.state.aero.circulation.Ma, ...
        deg2rad(wing.state.actuators.segments.rate(1,:)), abs_V_i, ...
        wing.state.geometry.ctrl_pt.c, wing.state.aero.unsteady.z(3,:) );
wing.state.aero.unsteady.c_m_c = wing.state.aero.unsteady.c_m_c + c_m_flap;

switch wing.config.actuator_2_type
    case 'none'
        % do nothing
    case 'custom'
        [c_L_act2,c_m_act2,c_D_act2,z2_dt] = wingCustomActuator(wing);
        wing.state.aero.unsteady.z2_dt = z2_dt;
        wing.state.aero.unsteady.c_L_act2 = c_L_act2;
        wing.state.aero.unsteady.c_L_c = wing.state.aero.unsteady.c_L_c ...
            + c_L_act2;
        wing.state.aero.unsteady.c_D = wing.state.aero.unsteady.c_D ...
            + c_D_act2;
        wing.state.aero.unsteady.c_m_c = wing.state.aero.unsteady.c_m_c ...
            + c_m_act2;
end

end