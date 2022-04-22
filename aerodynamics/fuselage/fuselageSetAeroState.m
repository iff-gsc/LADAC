function fuselage = fuselageSetAeroState( fuselage, xyz_cg )
% fuselageSetAeroState set state.aero struct in fuselage struct
% 
% Inputs:
%   fuselage        fuselage struct (see fuselageInit)
%   xyz_cg          center of gravity position in fuselage frame (3x1
%                   array), in m
% 
% Outputs:
%   fuselage        fuselage struct (see fuselageInit)
% 
% See also:
%   fuselageInit, fuselageStateInit, fuselageSetState
% 
% Literature:
%   [1] Schlichting, H., & Truckenbrodt, E. (2001). Aerodynamik des
%       Flugzeuges. Zweiter Band: Aerodynamik des Tragflügels (Teil II),
%       des Rumpfes, der Flügel-Rumpf-Anordnung und der Leitwerke. 3.
%       Auflage. Springer-Verlag Berlin Heidelberg.

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% set local inflow
fuselage = fuselageSetLocalInflow( fuselage, xyz_cg );

V_abs = vecnorm( fuselage.state.aero.local_inflow.V, 2, 1 );
V_A = mean(V_abs);
Ma = V_A ./ fuselage.state.external.atmosphere.a;

dx = abs( diff( fuselage.geometry.border_pos(1,:) ) );

beta_Ma = sqrtReal( 1 - Ma^2 );
compr_fac = 1 / beta_Ma;

radius = fuselage.geometry.width/2;
% volumes of frustums
volume = pi/3 * dx .* ( radius(1:end-1).^2 + radius(1:end-1).*radius(2:end) + radius(2:end).^2 );
total_volume = sum( volume );
volume23 = volume.^(2/3);
total_volume_23 = total_volume^(2/3);

b2 = fuselage.geometry.width_visc.^2;

% compressible correction, [1], eq. (9.80d)
alpha = fuselage.state.aero.local_inflow.alpha * compr_fac;
% reduction for higher angles
alpha = 0.5*sin(2*alpha);
beta = fuselage.state.aero.local_inflow.beta * compr_fac;
beta = 0.5*sin(2*beta);
T_unsteady = 1;

% [1], eq. (10.7) -> derivative
if fuselage.config.is_unsteady
    % time constant
    b12 = 0.45;
    c = max(fuselage.params.width);
    T_unsteady(:) = 1 / ( (2*V_A/c)*beta_Ma^2*b12 );
    alB2_dx = diff( alpha .* b2 ) ./ dx;
    beB2_dx = diff( beta .* b2 ) ./ dx;
else
    alB2_dx = diff( alpha .* b2 ) ./ dx;
    beB2_dx = diff( beta .* b2 ) ./ dx;
end

% local ram air
q = fuselage.state.external.atmosphere.rho/2 * V_abs.^2;

if fuselage.config.is_unsteady
    lift = - pi/2 * q .* alB2_dx .* dx;
    side = - pi/2 * q .* beB2_dx .* dx;
    fuselage.state.aero.R_Ab_i(3,:) = fuselage.state.aero.unsteady.alpha;
    fuselage.state.aero.R_Ab_i(2,:) = fuselage.state.aero.unsteady.beta;
    fuselage.state.aero.R_Ab_i(1,:) = - fuselage.params.C_D0 * q .* volume23;
    fuselage.state.aero.unsteady.alpha_dt(:) = 1/T_unsteady * ( lift - ...
    fuselage.state.aero.unsteady.alpha );
    fuselage.state.aero.unsteady.beta_dt(:) = 1/T_unsteady * ( side - ...
    fuselage.state.aero.unsteady.beta );
else
    % [1], eq. (10.7)
    fuselage.state.aero.R_Ab_i(3,:) = - pi/2 * q .* alB2_dx .* dx;
    fuselage.state.aero.R_Ab_i(2,:) = - pi/2 * q .* beB2_dx .* dx;
    fuselage.state.aero.R_Ab_i(1,:) = - fuselage.params.C_D0 * q .* volume23;
end

% local coefficients
fuselage.state.aero.C_XYZ_b_i(:) = fuselage.state.aero.R_Ab_i ./ ...
    repmat( q .* volume23/sum(volume23) * total_volume_23, 3, 1 );

% global
fuselage.state.aero.R_Ab(:) = sum( fuselage.state.aero.R_Ab_i, 2 );
fuselage.state.aero.C_XYZ_b(:) = fuselage.state.aero.R_Ab / ( total_volume_23 * mean(q) );
r_ref = fuselage.geometry.cntrl_pos - repmat( xyz_cg, 1, fuselage.n_segments );
fuselage.state.aero.Q_Ab(:) = sum (cross( r_ref, fuselage.state.aero.R_Ab_i ), 2 );

end