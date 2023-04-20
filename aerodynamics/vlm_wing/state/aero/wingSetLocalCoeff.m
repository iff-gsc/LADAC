function wing = wingSetLocalCoeff( wing )
% wingSetLocalCoeff sets the wing.state.aero.coeff_loc struct in a wing
% struct.
%   It computes computes the local aerodynamics coefficients for a spanwise
%   discretized wing based on the computed circulation and on the
%   specified airfoil.
% 
% Inputs:
% 	wing           	wing struct, see wingCreate
% 
% Outputs:
% 	wing           	wing struct, see wingCreate
% 
% Literature:
%   [1] Phillips, W. F., & Snyder, D. O. (2000). Modern adaption of
%       Prandtl's classic lifting-line theory. Jounal of Aircraft, 37(4),
%       662-670.
%   [2] Leishman, J. G., & Nguyen, K. Q. (1990). State-space representation
%       of unsteady airfoil behavior. AIAA journal, 28(5), 836-844.
% 
% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

zeta = wingGetDimLessSpanwiseLengthVector( wing.state.geometry.vortex );
% make normal_vector unit vector later
normal_vector = crossFast( wing.state.aero.circulation.v_i, zeta );
normal_vector_length = vecnorm(normal_vector,2,1);

% compute mean chord
c = wing.params.S/wing.params.b;

if ~wing.config.is_unsteady
    % Compute the aerodynamic force coefficients in aircraft frame
    % similar to [1], eq. 25.
    for i = 1:3
        normal_vector(i,:) = normal_vector(i,:) ./ normal_vector_length;
        wing.state.aero.coeff_loc.c_XYZ_b(i,:) = wing.state.aero.circulation.c_L ...
            .* normal_vector(i,:);
    end
    c_D = zeros(1,wing.n_panel);
    
    % to do: 'map' supported multiple segments/airfoils per wing but
    % 'analytic' does not
    switch wing.config.airfoil_method
        case 'analytic'
            % drag coefficient
            fcd = airfoilAnalytic0515Ma( wing.airfoil.analytic.wcd, wing.state.aero.circulation.Ma );
            c_D(:) = airfoilAnalytic0515AlCd( fcd, rad2deg(wing.state.aero.circulation.alpha_eff ) )';
            % local airfoil pitching moment coefficient w.r.t. local c/4
            fcm = airfoilAnalytic0515Ma( wing.airfoil.analytic.wcm, wing.state.aero.circulation.Ma );
            fcl = airfoilAnalytic0515Ma( wing.airfoil.analytic.wcl, wing.state.aero.circulation.Ma );
            [ c_L_alpha, alpha_0 ] = airfoilAnalytic0515ClAlphaMax( fcl, wing.state.aero.circulation.Ma );
            f_st = airfoilDynStallFst( wing.state.aero.circulation.c_L, c_L_alpha, rad2deg( wing.state.aero.circulation.alpha_eff ) - alpha_0 );
            wing.state.aero.coeff_loc.c_m_airfoil(:) = airfoilAnalyticBlCm( fcm, f_st, wing.state.aero.circulation.c_L );
        case 'simple'
            % drag coefficient
            c_D(:) = airfoilAnalyticSimpleCd( wing.airfoil.simple, ...
                wing.state.aero.circulation.alpha_eff );
            % local airfoil pitching moment coefficient w.r.t. local c/4
            wing.state.aero.coeff_loc.c_m_airfoil = airfoilAnalyticSimpleCm(wing.airfoil.simple,wing.state.aero.circulation.c_L);
    end
    
    % quasi-steady pitch damping derived from [2], eq. (A15) and (A13)
    % to do: consider wing downwash in dimensionless pitch rate
    wing.state.aero.coeff_loc.c_m_airfoil(:) = wing.state.aero.coeff_loc.c_m_airfoil ...
        - pi/8./sqrtReal(1-powerFast(wing.state.aero.circulation.Ma,2)) .* wing.state.aero.circulation.q;
    
    % flap moment
    wing.state.aero.coeff_loc.c_m_airfoil = wing.state.aero.coeff_loc.c_m_airfoil ...
        + airfoilFlapMoment( wing.state.aero.circulation.c_L_flap, wing.geometry.segments.flap_depth );
    
    % 2nd actuator coefficients
    c_D_act2 = zeros(size(c_D));
    c_m_act2 = zeros(size(c_D));
    switch wing.config.actuator_2_type
        case 'none'
            % do nothing
        case 'micro-tab'
            [ ~, c_D_act2(:), c_m_act2(:) ] = airfoilMicroTabDeltaCoeff( ...
                wing.airfoil.micro_tab, wing.state.aero.circulation, ...
                wing.state.actuators.segments.pos(2,:) );
    end
    c_D = c_D + c_D_act2;
    wing.state.aero.coeff_loc.c_m_airfoil = wing.state.aero.coeff_loc.c_m_airfoil ...
        + c_m_act2;
    
    % apply drag to force coefficients (in body frame)
    wing.state.aero.coeff_loc.c_XYZ_b = wing.state.aero.coeff_loc.c_XYZ_b + ...
        repmat( c_D, 3, 1 ) .* wing.state.aero.circulation.v_i_unit;
    
else
    
    % unsteady coefficients
    for i = 1:3
        normal_vector(i,:) = normal_vector(i,:)./normal_vector_length;
        wing.state.aero.coeff_loc.c_XYZ_b(i,:) = ( wing.state.aero.unsteady.c_L_c ...
            + wing.state.aero.unsteady.c_L_nc ) .* normal_vector(i,:) ...
            + wing.state.aero.unsteady.c_D .* wing.state.aero.circulation.v_i_unit(i,:);
    end
        % local airfoil pitching moment coefficient w.r.t. local c/4
        wing.state.aero.coeff_loc.c_m_airfoil(:) = wing.state.aero.unsteady.c_m_c ...
            + wing.state.aero.unsteady.c_m_nc;

end

% reference point for moment calculation in the center of the bound segment
r_ref = wing.state.geometry.vortex.pos(:,1:end-1) + diff(wing.state.geometry.vortex.pos,1,2)/2;

% compute moment coefficient distribution produced by force
% coefficients (w.r.t. wing origin)
wing.state.aero.coeff_loc.c_lmn_b(:) = crossFast( r_ref, wing.state.aero.coeff_loc.c_XYZ_b ) ...
    ./ repmat( [ wing.params.b; c; wing.params.b ], 1, wing.n_panel );
% contribution of airfoil and flap moment
wing.state.aero.coeff_loc.c_lmn_b(2,:) = wing.state.aero.coeff_loc.c_lmn_b(2,:) ...
    + wing.state.aero.coeff_loc.c_m_airfoil .* wing.geometry.ctrl_pt.c / c;

end