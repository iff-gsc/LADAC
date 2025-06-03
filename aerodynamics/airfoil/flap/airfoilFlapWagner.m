function [c_L_flap,alpha_deta,x_dt] = airfoilFlapWagner( V, c, C_L_alpha, E, x, eta )
% airfoilFlapWagner computes unsteady flap aerodynamics according to the
% Wagner function.
% 
% Inputs:
%   V           Airspeed (1xn vector), in m/s
%   c           Chord (1xn vector), in m
%   C_L_alpha   Lift curve slope (1xn vector), in 1/rad
%   E           Flap depth relative to the chord, dimensionless
%   x           State (2xn matrix)
%   eta         Flap deflection angle (1xn vector), in rad
% 
% Outputs:
%   c_L_flap	Unsteady flap lift coefficient (1xn vector)
%   alpha_deta  Static effective angle of attack due to flap deflection
%               (1xn vector), in 1/rad
%   x_dt        Time-derivative of the state x (2xn matrix)
% 
% Literature:
%   [1] Leishman, J. G. (1994). Unsteady lift of a flapped airfoil by
%       indicial concepts. Journal of Aircraft, 31(2), 288-297.
%       https://arc.aiaa.org/doi/pdf/10.2514/3.46486
% 
% Authors:
%   Yannic Beyer
% 
% See also:
%   unstAirfoilWagner

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2025 Yannic Beyer
%   Copyright (C) 2025 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

[c_L,~,~,x_dt]  = unstAirfoilWagner( V, c, C_L_alpha, 0.25, x, eta );

c_L_nc_alpha    = 0.5*C_L_alpha * eta;
c_L_nc          = E.*c_L_nc_alpha;
c_L_c           = (2-E).*(c_L-c_L_nc_alpha);

e           = 1-2*E;
e2          = powerFast(e,2);
sqrt_1_e2   = sqrtReal(1-e2);
acos_e      = acosReal(e);
F_10        = sqrt_1_e2 + acos_e;
alpha_deta  = F_10/pi;

c_L_flap 	= alpha_deta * (c_L_c + c_L_nc);

end