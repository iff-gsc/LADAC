function atmosphere = isAtmosphere( h ) %#codegen
% atmosphere computes the state of the air assuming norm conditions
%   The state of the air, density (rho), speed of sound (a), pressure (p),
%   and temperature (T), are computed according to the international
%   standard atmosphere.
%
% Inputs:
%   h           altitude above mean sea level (double), m
%
% Outputs:
%   Struct containing the following variables:
%   rho         air density (double), kg/m^3
%   a           air speed of sound (double), m/s
%   p           air pressure (double), Pa
%   T           air temperature (double), K
%   mu          air dynamic viscosity (double), kg/(m*s)
%
% See also: aerodynamicsMulticopter
%

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% get parameter (conditions at the ground and at the stratopause
T_0 = 288.15;
T_11 = 216.65;
rho_0 = 1.225;
rho_11 = 0.3639;
p_0 = 101325;
p_11 = 22633;
g = 9.80665;
kappa = 1.4;
dTdh = 0.0065;
R = 287.058;
mu_0 = 1.7894*10^(-5);

% computation of the physical quantities depending on the altitude
h(h<0) = 0;
if h <= 11000
    % troposhere
    rho = rho_0*(1-dTdh*(h/T_0))^(g/dTdh/R-1);
    p = p_0*(1-dTdh*(h/T_0))^(g/dTdh/R);
    T = T_0 - dTdh * h;
else
    % stratosphere
    rho = rho_11 * exp( -g/(R*T_11)*(h-11000) );
    p = p_11 * exp( -g/(R*T_11)*(h-11000) );
    T = T_11;
end

% speed of sound, m/s
a = sqrtReal(kappa*R*T);

% dynamic viscosity (Sutherland's Equation)
T_S = 110;
mu = mu_0 * ( T/T_0)^(3/2) * (T_0+T_S)/(T+T_S);

atmosphere.rho  = rho;
atmosphere.a    = a;
atmosphere.p    = p;
atmosphere.T    = T;
atmosphere.mu   = mu;

end