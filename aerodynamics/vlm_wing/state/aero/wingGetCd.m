function [ C_D, C_Di, c_D, c_Di ] = wingGetCd( wing )
% wingGetCd computes the drag coefficient of a wing struct.
%   It computes computes the global and local drag and induced drag for a
%   spanwise discretized wing based on the previously computed coefficients
%   in body frame.
% 
% Inputs:
% 	wing           	wing struct, see wingCreate
% 
% Outputs:
% 	C_D           	Drag coefficient (scalar)
%   C_Di            Induced drag coefficient (scalar)
%   c_D             Local drag coefficients (1xn_panel)
%   c_Di            Local induced drag coefficients (1xn_panel)
% 
% See also:
%   wingSetState, wingGetDrag

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2026 Yannic Beyer
%   Copyright (C) 2026 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

zeta = wingGetDimLessSpanwiseLengthVector( wing.state.geometry.line_25 );
% make normal_vector unit vector later
normal_vector = cross( wing.state.aero.circulation.v_i, zeta(:,:,1) );
normal_vector_length = vecnorm(normal_vector,2,1);

M_ba = dcmBaFromAeroAngles( wing.state.body.alpha, wing.state.body.beta );

% Unit vector in aerodynamic frame in drag direction
u_Da = [ -1; 0; 0 ];
u_Db = M_ba * u_Da;

V_rel = vecnorm( wing.state.aero.local_inflow.V_25 .* ...
    wing.state.aero.circulation.v_i, 2 )/wing.state.body.V;
V_rel_2 = powerFast(V_rel,2);

segment_span    = wingGetSegmentSpan(wing.state.geometry.line_25);

% Local lift coefficient vectors in body frame
c_L_b = zeros(3,wing.n_panel);
% Global lift coefficient vector in body frame
C_L_b = zeros(3,1);

c_L = wing.state.aero.circulation.c_L + wing.state.aero.unsteady.c_L_nc;
for i = 1:3
    normal_vector(i,:,:) = normal_vector(i,:,:) ./ normal_vector_length;
    c_L_b(i,:) = c_L .* normal_vector(i,:);
    C_L_b(i) = sum( sum( ...
        segment_span .* c_L_b(i,:,:) .* wing.state.geometry.ctrl_pt.c ...
        .* V_rel_2, 2), 3 ) / wing.params.S;
end

c_Di    = sum( c_L_b .* u_Db, 1 );
c_D     = sum( wing.state.aero.coeff_loc.c_XYZ_b .* u_Db, 1 );
C_Di    = sum( C_L_b .* u_Db );
C_D     = sum( wing.state.aero.coeff_glob.C_XYZ_b .* u_Db );

end