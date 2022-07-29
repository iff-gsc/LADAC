% flightPathBankAngle computes the flight-path bank angle mu_K based on
% conventions of ISO 1151 (or LN9300).
%   The DCM M_bg can be computed depeding on the Euler angles as well as
%   depending on flight-path angles (gamma, chi) and flight-path
%   aerodynamic angles (alpha_K, beta_K, mu_K): M_bg(Phi,Theta,Psi) =
%   M_bk(alpha_K,beta_K,mu_K)*M_kg(gamma,chi). By comparing an appropriate 
%   element of the matrix, the flight-path bank angle mu_K can be computed
%   [1, page 57-59].
% 
% Literature:
%   [1] Brockhaus, R. et al. (2011): Flugregelung. 3rd ed. Springer-Verlag.
%   [2]	Beyer, Y., Kuzolap, A., Steen, M., Diekmann, J. H., & Fezans, N. 
%       (2018). Adaptive Nonlinear Flight Control of STOL-Aircraft Based on
%       Incremental Nonlinear Dynamic Inversion. In 2018 Modeling and 
%       Simulation Technologies Conference (AIAA 6.2018-3257).
% 
% Inputs:
%   gamma           the scalar flight-path angle, rad
%   Phi             the scalar bank angle, rad
%   Theta           the scalar pitch angle, rad
%   beta_K          the scalar flight-path sideslip angle, rad
% 
% Outputs:
%   mu_K            the scalar flight-path bank angle, rad
% 
% See also: aeroAngles
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

function mu_K = flightPathBankAngle( gamma, Phi, Theta, beta_K ) %#codegen

% compute the flight-path bank angle according to [2, page 9]
mu_K = asinReal( (sin(gamma)*sin(beta_K)+sin(Phi)*cos(Theta)) / ...
    (cos(beta_K)*cos(gamma)) );

end