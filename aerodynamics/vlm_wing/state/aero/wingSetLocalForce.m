function wing = wingSetLocalForce( wing )
% wingSetLocalForce set array of vectors of local aerodynamic forces
% and moments of a wing struct.
% 
% Inputs:
%   wing        Wing struct (see wingCreate)
% 
%   wing        Wing struct (see wingCreate)
% 
% See also: wingSetGlobalForce, wingSetLocalCoeff, wingSetGlobalCoeff
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% chord
c = wing.params.S/wing.params.b;
% segment span
b_i = wingGetSegmentSpan( wing.geometry.vortex );
% absolute local airspeed
abs_V_loc = vecnorm( wing.state.aero.local_inflow.V_25 .* wing.state.aero.circulation.v_i, 2 );

% Compute factor for the aerodynamic force coefficients to get the forces.
% Note that the local airspeed is multiplied because the local
% aerodynamic coefficients are normalized by the local airspeed.
factor =  powerFast(abs_V_loc,2) * 0.5 * wing.state.external.atmosphere.rho ...
    .* b_i .* wing.geometry.ctrl_pt.c;

wing.state.aero.force_loc.M_i_b = wing.state.aero.coeff_loc.c_m_airfoil .* factor .* wing.geometry.ctrl_pt.c;

% Compute factor for the aerodynamic force coefficients to get the forces.
% Note that the local airspeed is multiplied because the local
% aerodynamic coefficients are normalized by the local airspeed.
factor3 =  repmat( factor, 3, 1 );

wing.state.aero.force_loc.R_i_b = wing.state.aero.coeff_loc.c_XYZ_b .* factor3;

% Compute factor for the aerodynamic moment coefficients to get the moment.
factor3 =  factor3 .* repmat( [wing.params.b;c;wing.params.b], 1, wing.n_panel );

wing.state.aero.force_loc.Q_i_b = wing.state.aero.coeff_loc.c_lmn_b .* factor3;

end