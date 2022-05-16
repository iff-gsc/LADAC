function LMN_i_b = wingGetLocalMoment( wing )
% wingGetLocalMoment returns an array of vectors of local aerodynamic
% moments of the discretized wing.
% 
% Inputs:
%   wing        Wing struct (see wingCreate)
% 
% Outputs:
%   LMN_i_b     Local aerodynamic moments (3xn array) with respect to the
%               wing origin in body frame (b), in Nm
% 
% See also: wingSetLocalMomentCoeff, wingGetLocalForce, wingGetGlobalMoment
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
% local absolute airspeed
abs_V_loc = vecnorm( wing.state.aero.local_inflow.V_25, 2 );
% air density
rho = wing.state.external.atmosphere.rho;

% Compute factor for the aerodynamic moment coefficients to get the moment.
% Note that the local airspeed is multiplied because the local
% aerodynamic coefficients are normalized by the local airspeed.
b = wing.params.b;
c = wing.params.S/wing.params.b;
factor =  repmat( abs_V_loc.^2 * 0.5 * rho .* b_i .* c_i, 3, 1) ...
    .* repmat( [b;c;b], 1, wing.n_panel );
LMN_i_b = wing.state.aero.coeff_loc.c_lmn_b .* factor;

end