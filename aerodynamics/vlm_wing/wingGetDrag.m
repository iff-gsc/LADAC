function [ Drag, Drag_i, drags, drags_i ] = wingGetDrag( wing )
% wingGetDrag computes the drag of a wing struct.
%   It computes computes the global and local drag and induced drag for a
%   spanwise discretized wing based on the computed local coefficients.
% 
% Inputs:
% 	wing           	wing struct, see wingCreate
% 
% Outputs:
% 	Drag           	Drag force (scalar), in N
%   Drag_i          Induced drag force (scalar), in N
%   drags           Local drag forces (1xn_panel), in N
%   drags_i         Local induced drag forces (1xn_panel), in N
% 
% See also:
%   wingSetState, wingGetCd

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2026 Yannic Beyer
%   Copyright (C) 2026 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

b_i = wingGetSegmentSpan( wing.geometry.line_25 );
% Absolute local airspeed
abs_V_loc = vecnorm( wing.state.aero.local_inflow.V_25 ...
    .* wing.state.aero.circulation.v_i, 2 );

% Compute factor for the aerodynamic force coefficients to get the force
Factor =  wing.state.body.V^2 * 0.5 * ...
    wing.state.external.atmosphere.rho * wing.params.S;

factor =  abs_V_loc.^2 * 0.5 * wing.state.external.atmosphere.rho ...
    .* b_i .* wing.geometry.ctrl_pt.c;

[ C_D, C_Di, c_D, c_Di ] = wingGetCd( wing );

Drag        = C_D * Factor;
Drag_i      = C_Di * Factor;
drags       = c_D .* factor;
drags_i     = c_Di .* factor;

end