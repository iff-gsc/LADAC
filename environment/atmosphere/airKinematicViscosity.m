function nu = airKinematicViscosity( T, rho ) %#codegen
% airKinematicViscosity computes the kinematic viscosity of air based on
%   Sutherland's law.
% 
% Inputs:
%   T           Temperatur (scalar), in K
%   rho         Density (scalar), in kg/m^3
% 
% Outputs:
%   nu          Kinematic viscosity (scalar), in m^2/s
% 
% See also: isAtmosphere
%

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% Sutherland's constant, in K
C = 120;
% reference temperature, in K
T_0 = 291.15;
% reference viscosity, in Pa*s
mu_0 = 18.27e-6;

% dynamic viscosity
mu = mu_0 * (T_0+C)/(T+C) * (T/T_0)^1.5;
% kinematic viscosity
nu = rho * mu;

end