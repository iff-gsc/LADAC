function wing = wingSetGlobalCoeff( wing ) %#codegen
% wingSetGlobalCoeff sets the wing.state.aero.coeff_glob struct in a wing
% struct.
% 
% Inputs:
% 	wing           	wing struct, see wingCreate
% 
% Outputs:
% 	wing           	wing struct, see wingCreate
%
% See also: wingSetLocalCoeff
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

V_rel = vecnorm( wing.state.aero.local_inflow.V_25 .* wing.state.aero.circulation.v_i, 2 )/wing.state.body.V;
V_rel_2 = powerFast(V_rel,2);

segment_span    = wingGetSegmentSpan(wing.state.geometry.line_25);

% A trapezoidal integration could give more accurate results than this
% panel-wise constant summation
for i = 1:3
    wing.state.aero.coeff_glob.C_XYZ_b(i) = sum( sum( ...
        segment_span .* wing.state.aero.coeff_loc.c_XYZ_b(i,:,:) .* wing.state.geometry.ctrl_pt.c .* V_rel_2 ...
        , 2), 3 ) / wing.params.S;
    wing.state.aero.coeff_glob.C_lmn_b(i) = sum( sum( ...
        wing.state.aero.coeff_loc.c_lmn_b(i,:,:) ...
        .* segment_span  .* wing.state.geometry.ctrl_pt.c .* V_rel_2 ...
        , 2), 3 ) / wing.params.S;
end

end