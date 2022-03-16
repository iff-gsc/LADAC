function [ XYZ_i_b ] = wingGetLocalForce( wing )
% wingGetLocalForce returns an array of vectors of local aerodynamic forces
% of a wing struct.
% 
% Inputs:
%   wing        Wing struct (see wingCreate)
% 
% Outputs:
%   XYZ_i_b     Local aerodynamic forces (3xn array) in body frame (b), in N
% 
% See also: wingSetLocalCoeff, wingGetLocalMoment, wingGetGlobalForce
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% segment span
b_i = wingGetSegmentSpan( wing.geometry.vortex );
% local chord
c_i = wing.geometry.ctrl_pt.c;
% absolute local airspeed
abs_V_loc = vecnorm( wing.state.aero.local_inflow.V, 2 );
% air density
rho = wing.state.external.atmosphere.rho;
% local aerodynamic force coefficients
c_XYZ_b = wing.state.aero.coeff_loc.c_XYZ_b;

% Compute factor for the aerodynamic force coefficients to get the force.
% Note that the local airspeed is multiplied because the local
% aerodynamic coefficients are normalized by the local airspeed.
factor =  repmat( abs_V_loc.^2 * 0.5 * rho .* b_i .* c_i, 3, 1 );
XYZ_i_b = c_XYZ_b .* factor;

end