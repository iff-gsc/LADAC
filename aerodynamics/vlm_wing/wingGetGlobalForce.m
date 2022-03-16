function XYZ_b = wingGetGlobalForce( wing )
% wingGetGlobalForce returns the total/global aerodynamic force vector
% of the discretized wing.
% 
% Inputs:
%   wing        Wing struct (see wingCreate)
% 
% Outputs:
%   XYZ_b       Total/global aerodynamic force vector (3x1 array) in body
%               frame (b), in N
% 
% See also: wingSetGlobalCoeff, wingGetGlobalMoment, wingGetLocalForce
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% air density
rho = wing.state.external.atmosphere.rho;
% global aerodynamic force coefficients
C_XYZ_b = wing.state.aero.coeff_glob.C_XYZ_b;
% global absolute airspeed
V = wing.state.body.V;
% wing reference area
S = wing.params.S;

% Compute factor for the aerodynamic force coefficients to get the force.
factor =  V.^2 * 0.5 * rho * S;
XYZ_b = C_XYZ_b .* factor;

end