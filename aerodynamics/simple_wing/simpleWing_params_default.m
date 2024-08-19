%  ** simple wing aerodynamics parameters (default) **

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% values according to [1]
% maximum angle of attack for linear C_L, deg
wing.polar.params.alpha_linend = 10;
% angle of attack for maximum C_L (stall), deg
wing.polar.params.alpha_max = 15;
% zero angle of attack lift coefficient, -
wing.polar.params.C_L0 = 0;
% maximum lift coefficient (C_L), -
wing.polar.params.C_Lmax = 0.80;
% angle of attack for minimum C_L after stall, deg
wing.polar.params.alpha_min = 22;
% minimum C_L after stall, -
wing.polar.params.C_Lmin = 0.7;
% angle of attack for second maximum C_L after stall, deg
wing.polar.params.alpha_max2 = 40;
% second maximum C_L after stall, -
wing.polar.params.C_Lmax2 = 0.8;
% drag coefficient for zero angle of attack, -
wing.polar.params.C_D0 = 0.1;
% drag coefficient for 90deg angle of attack, -
wing.polar.params.C_Dmax = 1.533;
% drag coeffecient at stall, -
wing.polar.params.C_Dkink = 0.35;
% angle of attack for drag coefficient at stall, deg
wing.polar.params.alpha_kink = 17;


% span, m
wing.geometry.b = 0.84;
% Oswald's efficiency factor, -
wing.geometry.e = 0.8;
% mean chord, m
wing.geometry.c = 0.25;
% sweep, rad
wing.geometry.phi = 18*pi/180;
% taper, -
wing.geometry.z = 0.47;
% dihedral angle, rad
wing.geometry.v = 0;


% maximum C_L for 90deg sideslip angle, -
wing.lateral_polar.c_Lbeta90max = 0.5;
% derivative dC_L/dalpha|beta=90deg,1/rad
wing.lateral_polar.c_Lbeta90alpha0 = 0.3/(pi/4);


% y beginning of the flap relative to the span, -
wing.flap.eta_0 = 0.65;
% depth of the flap relative to the chord, -
wing.flap.lambda_K = 0.22;
% empirical flap effectiveness reduction factor, -
wing.flap.x = 0.5;
% drag coefficient due to elevon deflection
wing.flap.C_D_dEta = 0;
% effective flap deflection angle scaling factor (see 
% simpleWingGetEffFlapAng), -
% If the effective flap deflection angle should not be used, set
% is_eff_scaling 0 (Simulink does not like logical) or do not define the
% fields.
wing.flap.eff_scaling = 1;
wing.flap.is_eff_scaling = 0;



% Pitching moment coefficient derivative with respect to the dimensionless
% pitching rate ( Omega_y = q/(c*V), in 1 [1, p. 93] ).
% This derivative can be estimated from [1, p. 93], where it is defined as
% (d_C_m/d_Omega_y)_0.
wing.C_m_dOmega_0 = -0.35;