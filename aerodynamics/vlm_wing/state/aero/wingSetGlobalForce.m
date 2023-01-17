function wing = wingSetGlobalForce( wing, pos_ref_c )
% wingSetGlobalForce set total/global aerodynamic force and moment vector
% of the discretized wing.
% 
% Inputs:
%   wing        Wing struct (see wingCreate)
% 
% Outputs:
%   wing        Wing struct (see wingCreate)
% 
% See also: wingSetGlobalCoeff, wingSetLocalForce, wingSetLocalCoeff
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% wing mean chord
c = wing.params.S/wing.params.b;

% Compute factor for the aerodynamic force coefficients to get the force.
factor =  powerFast(wing.state.body.V,2) * 0.5 * ...
    wing.state.external.atmosphere.rho * wing.params.S;

wing.state.aero.force_glob.R_b(:) = wing.state.aero.coeff_glob.C_XYZ_b .* factor;

% Compute factor for the aerodynamic moment coefficients to get the moment.
factor =  factor .* [wing.params.b;c;wing.params.b];

wing.state.aero.force_glob.Q_origin_b(:) = wing.state.aero.coeff_glob.C_lmn_b .* factor;

[ ~, wing.state.aero.force_glob.Q_ref_b(:) ] = forceMomentTransform( ...
    wing.state.aero.force_glob.R_b, wing.state.aero.force_glob.Q_origin_b, ...
    wing.geometry.origin - pos_ref_c, eye(3) );
end