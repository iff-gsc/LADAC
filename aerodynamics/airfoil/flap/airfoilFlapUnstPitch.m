function [c_m_c,z_3_dt] = airfoilFlapUnstPitch(flap_depth,delta,Ma,delta_dt,V,c,z_3) %#codegen
% airfoilFlapPitch Unsteady pitching moment coefficient model for a flap
%   according to [1].
% 
% Inputs:
%   flap_depth  Flap depth relative to the chord, dimensionless
%   delta       Flap deflection angle (1xN array), in rad
%   Ma          Mach number (1xN array), dimensionless
%   delta_dt	Flap deflection rate (1xN array), in rad/s
%   V           airspeed (1xN array), in m/s
%   c           airfoil chord (1xN array), in m
%   z           state variable (1xN array)
% 
% Outputs:
%   c_m_c       Circulatory part of the unsteady pitching moment
%               coefficient due to the flap (1xN array), dimensionless
%   z_dt        Time derivative of the state variable (1xN array)
% 
% Literature:
%   [1] Hariharan, N., & Leishman, J. G. (1996). Unsteady aerodynamics of a
%       flapped airfoil in subsonic flow by indicial concepts. Journal of
%       Aircraft, 33(5), 855-868.

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2023 Yannic Beyer
%   Copyright (C) 2023 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

b_3     = 0.1393;

beta    = sqrtReal(1-powerFast(Ma,2));

F = airfoilFlapEffectiveness(flap_depth);

% [1], eq. (51)
delta_qs_M = -(F.F_4+F.F_10)./(2*pi.*beta).*delta ...
    -((2*F.F_1-2.*F.F_8-(2*F.e+1).*F.F_4+F.F_11)./(8*pi*beta)) .* c./V.* delta_dt;

% [1], eq. (49)
z_3_dt = -(2*V./c)*b_3.*powerFast(beta,2).*z_3 + delta_qs_M;

% [1], eq. (50)
c_m_c = pi*beta*b_3.*(2*V./c).*z_3;

end

