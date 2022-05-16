function M_i_b = wingGetLocalAirfoilMoment( wing )
% wingGetLocalAirfoilMoment returns an array of vectors of local aerodynamic
% pitching moments of the discretized wing w.r.t. the local c/4 position.
% 
% Inputs:
%   wing        Wing struct (see wingCreate)
% 
% Outputs:
%   M_i_b       Local airfoil pitching moments (1xn array) with respect to
%               the local c/4 positions, in Nm
% 
% See also: wingSetLocalMomentCoeff, wingGetLocalForce, wingGetGlobalMoment

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
% local airspeed vector
V_rot = wing.state.aero.local_inflow.V_25;
% local absolute airspeed
abs_V_rot = vecnorm( V_rot, 2 );
% air density
rho = wing.state.external.atmosphere.rho;

% Compute factor for the aerodynamic moment coefficients to get the moment.
% Note that the local airspeed is multiplied because the local
% aerodynamic coefficients are normalized by the local airspeed.
c = wing.params.S/wing.params.b;
factor =  abs_V_rot.^2 * 0.5 * rho .* b_i .* c_i * c;
M_i_b = wing.state.aero.coeff_loc.c_m_airfoil .* factor;

end