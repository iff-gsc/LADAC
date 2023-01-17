function S_Omega = drydenPowerSpectralDensity( sigma, L, Omega, axis )
%#codegen
% drydenPowerSpectralDensity computes the power spectral density of the
%   Dryden spectrum for the specified direction.
% 
% Inputs:
%   sigma       root-mean-square wind turbulence intensity, in m/s
%   L           wind scale length, in m
%   Omega       spatial frequency Omega =  omega/V, in rad/m
%   axis        a string specifying the direction ('u', 'v' or 'w')
% 
% Outputs:
%   S_Omega     power spectral density, in (m^2/s^2)/(1/m)
% 
% Literature:
%   [1] Gage, S. (2003): Creating a Unified Graphical Wind Turbulence Model
%       from Multiple Specifications. In: AIAA Modeling and Simulation
%       Technologies Conference and Exhibit. AIAA 2003-5529. Austin, Texas.
%   [2] Wang, S.-T. (1980): Atmospheric Turbulence Simulation Techniques
%       With Application to Flight Analysis. NASA Contractor Report 3309.
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

switch axis
    case 'u'
        S_Omega = 2/pi * sigma^2 * L * 1 ./ ( 1+ L^2*Omega.^2 );
    case 'v'
        S_Omega = 1/pi * sigma^2 * L .* ( 1 + 3*(L*Omega).^2 ) ...
        ./ ( 1 + (L*Omega).^2 ).^2;
    case 'w'
        S_Omega = 1/pi * sigma^2 * L .* ( 1 + 3*(L*Omega).^2 ) ...
        ./ ( 1 + (L*Omega).^2 ).^2;
end
        