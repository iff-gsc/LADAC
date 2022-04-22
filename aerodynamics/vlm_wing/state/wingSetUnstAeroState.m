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
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% init
alpha_inf_0 = zeros( 1, wing.n_panel );

% use raw inflow in the first step and take into account downwash
% afterwards
abs_V_i = vecnorm( wing.state.aero.local_inflow.V, 2 );
abs_V_i_3 = repmat( abs_V_i, 3, 1 );
alpha_dt = aeroAnglesDeriv( abs_V_i_3 .* wing.interim_results.u_n, ...
    repmat( vecnorm( wing.state.aero.local_inflow.V_dt, 2 ), 3, 1 ) .* wing.interim_results.u_n );

% dimensionless pitch rate [1], Nomenclature
q = alpha_dt .* wing.state.geometry.ctrl_pt.c ./ abs_V_i;

% to do: explain why aerodynamic center is at 0.25c
x_ac = 0.25 * ones( 1, wing.n_panel );

switch wing.config.airfoil_method
    case 'analytic'

        % get coefficients of analytic functions for different Mach numbers
        fcl = airfoilAnalytic0515Ma( wing.airfoil.analytic.wcl, wing.state.aero.circulation.Ma, wing.airfoil.analytic.ncl, wing.airfoil.analytic.ocl );
        fcd = airfoilAnalytic0515Ma( wing.airfoil.analytic.wcd, wing.state.aero.circulation.Ma, wing.airfoil.analytic.ncd, wing.airfoil.analytic.ocd );
        fcm = airfoilAnalytic0515Ma( wing.airfoil.analytic.wcm, wing.state.aero.circulation.Ma, wing.airfoil.analytic.ncm, wing.airfoil.analytic.ocm );

        % get points on lift curve
        [c_L_alpha_max,alpha_0] = airfoilAnalytic0515ClAlphaMax( fcl, wing.state.aero.circulation.Ma(:) );

        % effective angle of attack for an equivalent uncambered airfoil
        alpha_inf_0(:) = wing.state.aero.circulation.alpha_eff - deg2rad(alpha_0)';

        % reduce effective dimensionless pitch rate due to wing downwash
        factor_3d = max( 0.1, min( 1, 1 - wing.state.aero.circulation.alpha_ind ./ alpha_inf_0 ) );
        idx_nzero = abs(alpha_inf_0)>1e-5;
        q(idx_nzero) = q(idx_nzero) .* factor_3d(idx_nzero);
        
        % linear unsteady aerodynamics (potential flow) according to [1]
        [ wing.state.aero.unsteady.c_L_c, wing.state.aero.unsteady.c_m_c, ...
            wing.state.aero.unsteady.c_L_nc, wing.state.aero.unsteady.c_m_nc, ...
            wing.state.aero.unsteady.alpha_eff, wing.state.aero.unsteady.x_dt ...
            ] = ...
            unstAirfoilAeroFast( abs_V_i, wing.state.aero.circulation.Ma, ...
            wing.state.geometry.ctrl_pt.c, rad2deg(c_L_alpha_max)', x_ac, ...
            wing.state.aero.unsteady.x, alpha_inf_0, q );
        
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
            airfoilDynStall( wing.state.aero.unsteady.X, wing.state.aero.circulation.Ma(:), ...
            C_N_p(:), c_m_p(:), wing.state.aero.unsteady.alpha_eff(:), fcl, fcd, fcm, ...
            abs_V_i(:), wing.state.geometry.ctrl_pt.c(:), wing.state.aero.circulation.alpha_eff(:), ...
            wing.state.aero.unsteady.tau_v(:) );

        % avoided new variable
        wing.state.aero.unsteady.c_m_c = wing.state.aero.unsteady.c_m_c - wing.state.aero.unsteady.c_m_nc;
    
    case 'simple'

        % effective angle of attack for an equivalent uncambered airfoil
        alpha_inf_0(:) = wing.state.aero.circulation.alpha_eff - deg2rad(wing.airfoil.simple.alpha_0);
        
        % reduce effective pitch rate due to wing downwash
        factor_3d = max( 0.1, min( 1, 1 - wing.state.aero.circulation.alpha_ind ./ alpha_inf_0 ) );
        idx_nzero = abs(alpha_inf_0)>1e-5;
        q(idx_nzero) = q(idx_nzero) .* factor_3d(idx_nzero);
        
        % clean airfoil coefficients
        c_L_alpha = wing.airfoil.simple.c_L_alpha ./ ...
            sqrtReal(1-wing.state.aero.circulation.Ma.^2);
        
        [ wing.state.aero.unsteady.c_L_c, wing.state.aero.unsteady.c_m_c, ...
            wing.state.aero.unsteady.c_L_nc, wing.state.aero.unsteady.c_m_nc, ...
            wing.state.aero.unsteady.alpha_eff, wing.state.aero.unsteady.x_dt ] ...
            = unstAirfoilAeroFast( abs_V_i, wing.state.aero.circulation.Ma, ...
            wing.state.geometry.ctrl_pt.c, c_L_alpha, x_ac, wing.state.aero.unsteady.x, ...
            alpha_inf_0, q );
        
        c_D_st = airfoilAnalyticSimpleCd( wing.airfoil.simple, alpha_inf_0 );
        c_L_unst = wing.state.aero.unsteady.c_L_c + wing.state.aero.unsteady.c_L_nc;
        wing.state.aero.unsteady.c_D(:) = unstAirfoilAeroCd( c_D_st, c_L_unst, ...
            alpha_inf_0, wing.state.aero.unsteady.alpha_eff );
        
end

% rotate induced velocity unit vector
wing.state.aero.unsteady.v_i = rodrigues_rot( wing.state.aero.circulation.v_i, ...
    wing.state.aero.circulation.rot_axis, alpha_inf_0 - wing.state.aero.unsteady.alpha_eff );

% flap
[ wing.state.aero.unsteady.c_L_c_flap, wing.state.aero.unsteady.z_dt ] = airfoilFlapUnstLift( ...
    abs_V_i,wing.state.aero.circulation.Ma, wing.state.geometry.ctrl_pt.c, ...
    wing.state.aero.circulation.delta_qs, wing.state.aero.unsteady.z );
wing.state.aero.unsteady.c_L_c = wing.state.aero.unsteady.c_L_c ...
    + wing.state.aero.unsteady.c_L_c_flap;
wing.state.aero.unsteady.c_m_c = wing.state.aero.unsteady.c_m_c ...
    + airfoilFlapMoment( wing.state.aero.unsteady.c_L_c_flap, wing.geometry.segments.flap_depth );

switch wing.config.actuator_2_type
    case 'none'
        % do nothing
    case 'micro-tab'
        % 2nd actuator with first order delay
        % todo: time constant is not varied with reduced frequency
        wing.state.aero.unsteady.z2_dt = airfoilMicroTabStateDeriv( ...
            wing.airfoil.micro_tab, wing.state.aero.unsteady.z2, ...
            wing.state.actuators.segments.pos(2,:) );

        [ c_L_act2, c_D_act2, c_m_act2 ] = airfoilMicroTabDeltaCoeff( ...
            wing.airfoil.micro_tab, wing.state.aero.circulation, ...
            wing.state.aero.unsteady.z2 );
        
        wing.state.aero.unsteady.c_L_c = wing.state.aero.unsteady.c_L_c ...
            + c_L_act2;
        wing.state.aero.unsteady.c_D = wing.state.aero.unsteady.c_D ...
            + c_D_act2;
        wing.state.aero.unsteady.c_m_c = wing.state.aero.unsteady.c_m_c ...
            + c_m_act2;
end

end