function LMN_b = wingGetGlobalMoment( wing )
% wingGetGlobalMoment returns the total/global aerodynamic force vector
% of a wing struct.
% 
% Inputs:
%   wing        Wing struct (see wingCreate)
% 
% Outputs:
%   LMN_b       Total/global aerodynamic moment vector (3x1 array) in body
%               frame (b) w.r.t. wing origin, in Nm
% 
% See also: wingSetGlobalCoeff, wingGetGlobalForce, wingGetLocalMoment
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% air density
rho = wing.state.external.atmosphere.rho;
% global absolute airspeed
V = wing.state.body.V;
% wing reference area
S = wing.params.S;
% wing span
b = wing.params.b;
% wing mean chord
c = wing.params.S/wing.params.b;

% Compute factor for the aerodynamic moment coefficients to get the moment.
factor =  V.^2 * 0.5 * rho * S;
LMN_b = wing.state.aero.coeff_glob.C_lmn_b .* factor .* [b;c;b];

end