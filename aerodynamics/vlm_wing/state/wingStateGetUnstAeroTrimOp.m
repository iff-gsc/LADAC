function [x,X,z,z2] = wingStateGetUnstAeroTrimOp( wing_state, wing_airfoil, wing_config )

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

%% x (from wingSetUnstAeroState and unstAirfoilAeroFast)

n_panel = size( wing_state.aero.unsteady.x, 2 );

%%%
c_L_alpha = 0; alpha_inf_0 = 0; c_L_alpha_max = 0; alpha_0 = 0; fcl = 0;
%%%

switch wing_config.airfoil_method
    case 'analytic'
        % get coefficients of analytic functions for different Mach numbers
        fcl = airfoilAnalytic0515Ma( wing_airfoil.analytic.wcl, wing_state.aero.circulation.Ma );
        % get points on lift curve
        [c_L_alpha_deg,alpha_0] = airfoilAnalytic0515ClAlphaMax( fcl, wing_state.aero.circulation.Ma );
        c_L_alpha = rad2deg(c_L_alpha_deg);
        % effective angle of attack for an equivalent uncambered airfoil
        alpha_inf_0 = wing_state.aero.circulation.alpha_eff - deg2rad(alpha_0);
        alpha_0_rad = deg2rad(alpha_0);
    case 'simple'
        % effective angle of attack for an equivalent uncambered airfoil
        alpha_inf_0 = wing_state.aero.circulation.alpha_eff - deg2rad(wing_airfoil.simple.alpha_0);
        % clean airfoil coefficients
        c_L_alpha = wing_airfoil.simple.c_L_alpha ./ ...
            sqrtReal(1-wing_state.aero.circulation.Ma.^2);
end

abs_V_i = vecnorm( wing_state.aero.local_inflow.V_75, 2 );

x_ac = 0.25 * ones( 1, n_panel );

% assume that aerodynamics is steady state
x_dt = zeros( size( wing_state.aero.unsteady.x ) );
% assume that (dimensionless) pitch rate is zero (steady state)
q = zeros( 1, n_panel );

x = zeros( size( wing_state.aero.unsteady.x ) );

[~,~,~,~,~,~,A,B_alpha,~] = unstAirfoilAeroFast( ...
    abs_V_i, wing_state.aero.circulation.Ma, wing_state.geometry.ctrl_pt.c, c_L_alpha, x_ac, x, alpha_inf_0 + wing_state.aero.circulation.alpha_ind, q );

x = (x_dt - B_alpha.*repmat(alpha_inf_0 + wing_state.aero.circulation.alpha_ind,8,1)) ./ A;

% for i = 1:size(wing_state.aero.unsteady.x,2)
%     
%     [A,B,C,D] = unstAirfoilAero( abs_V_i(i), wing_state.aero.circulation.Ma(i), wing_state.geometry.ctrl_pt.c(i), c_L_alpha(i), x_ac(i) );
%     
%     u = [ alpha_inf_0(i); q(i) ];
%     x(i) = A \ ( x_dt(i) - B * u );
%     
% end

%% X (from airfoilDynStall)

switch wing_config.airfoil_method
    case 'analytic'
        
        [ c_L_c, ~, c_L_nc, ~, ~, ~ ] = ...
            unstAirfoilAeroFast( abs_V_i, wing_state.aero.circulation.Ma, ...
            wing_state.geometry.ctrl_pt.c, c_L_alpha, x_ac, ...
            x, alpha_inf_0 + wing_state.aero.circulation.alpha_ind, q );
        
        c_L_c = c_L_c - c_L_alpha .* wing_state.aero.circulation.alpha_ind;


        C_N_p = ( c_L_c + c_L_nc );
        
        X1 = C_N_p;
        
        while true
        
            alpha_f = X1 ./ c_L_alpha;
            c_L_st_f = airfoilAnalytic0515AlCl( fcl, [ rad2deg(alpha_f+alpha_0_rad); wing_state.aero.circulation.Ma ] );

            f_s = airfoilDynStallFst( c_L_st_f, deg2rad(c_L_alpha), rad2deg(alpha_f) );

            c_v = zeros( size(f_s) );

            u = [ C_N_p; 1-f_s; c_v ];
            
            % assume that aerodynamics is steady state
            X = u;
            
            if sum(abs(X1-X(1,:))) < 1e-8
                break;
            else
                X1 = X(1,:);
            end
            
        end
    case 'simple'
        X = zeros( size( wing_state.aero.unsteady.X ) );
    otherwise %%%
        X = zeros( size( wing_state.aero.unsteady.X ) );
end


%% z and z2 (to do: zero is probably only correct for zero deflection)

z = zeros( size( wing_state.aero.unsteady.z ) );

z2 = zeros( size( wing_state.aero.unsteady.z2 ) );



end