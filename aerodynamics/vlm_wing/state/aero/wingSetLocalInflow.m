function wing = wingSetLocalInflow( wing, pos_ref_c )
% wingSetLocalInflow sets the wing.state.aero.local_inflow struct of a wing
% struct.
%   The local inflow for each control point is computed. The rotation of
%   the rigid body, the structural motion and the local wind are
%   considered.
% 
% Inputs:
% 	wing           	wing struct, see wingCreate
%   pos_ref_c      	vehicle reference position (for rigid body parameters)
%                   in c frame (3x1 array), in m
% 
% Outputs:
% 	wing           	wing struct, see wingCreate
% 
% Literature:
%   [1] Leishman, J. G., & Nguyen, K. Q. (1990). State-space representation
%       of unsteady airfoil behavior. AIAA journal, 28(5), 836-844.

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% reference point position w.r.t. the wing origin
pos_ref_wing = pos_ref_c - wing.geometry.origin;

% local airspeed from rotation
r_cntrl_ref = wing.state.geometry.ctrl_pt.pos ...
    - repmat( pos_ref_wing, 1, size(wing.state.geometry.ctrl_pt.pos,2) );
V_Ab = dcmBaFromAeroAngles( wing.state.body.alpha, wing.state.body.beta ) ...
    * [ wing.state.body.V; 0; 0 ];

% total local airspeed including local wind (external) and structure motion
wing.state.aero.local_inflow.V_75 = velocityFromRot( V_Ab, wing.state.body.omega, r_cntrl_ref ) ...
    + wing.state.geometry.ctrl_pt_dt.pos - wing.state.external.V_Wb;

% approx. airspeed and angle of attack at 25% chord
wing.state.aero.local_inflow.V_25 = wing.state.aero.local_inflow.V_75;
wing.state.aero.local_inflow.V_25(3,:) = ...
    wing.state.aero.local_inflow.V_25(3,:) - ...
    wing.state.geometry.ctrl_pt_dt.local_incidence .* wing.geometry.ctrl_pt.c/2;
wing.state.aero.local_inflow.V_25(:) = ...
    wing.state.aero.local_inflow.V_25 - wing.state.external.V_Wb_dt ...
    .* repmat( wing.geometry.ctrl_pt.c/2 ./ vecnorm( wing.state.aero.local_inflow.V_75, 2, 1 ), 3, 1 );

end