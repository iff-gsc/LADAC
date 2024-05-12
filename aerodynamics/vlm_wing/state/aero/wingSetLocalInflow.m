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
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

n_panel_x = 1;

% reference point position w.r.t. the wing origin
pos_ref_wing = pos_ref_c - wing.geometry.origin;

% local airspeed from rotation
r_cntrl_ref = wing.state.geometry.ctrl_pt.pos ...
    - repmat( pos_ref_wing, 1, wing.n_panel, n_panel_x );
r_cntrl_ref_25 = r_cntrl_ref;
r_cntrl_ref_25(1,:) = r_cntrl_ref_25(1,:) + wing.geometry.ctrl_pt_lever;
V_Ab = dcmBaFromAeroAngles( wing.state.body.alpha, wing.state.body.beta ) ...
    * [ wing.state.body.V; 0; 0 ];

% total local airspeed including local wind (external) and structure motion
wing.state.aero.local_inflow.V_75(:,:) = velocityFromRot( V_Ab, wing.state.body.omega, r_cntrl_ref(:,:) ) ...
    + wing.state.geometry.ctrl_pt_dt.pos(:,:) - wing.state.external.V_Wb(:,:);

% approx. airspeed and angle of attack at 25% chord
wing.state.aero.local_inflow.V_25(:,:) = velocityFromRot( V_Ab, wing.state.body.omega, r_cntrl_ref_25(:,:) ) ...
    + wing.state.geometry.ctrl_pt_dt.pos(:,:) - wing.state.external.V_Wb(:,:);
for i = 1:n_panel_x
    wing.state.aero.local_inflow.V_25(3,:,i) = ...
        wing.state.aero.local_inflow.V_25(3,:,i) - ...
        wing.state.geometry.ctrl_pt_dt.local_incidence(:,:,i) .* wing.geometry.ctrl_pt_lever(:,:,i);
    wing.state.aero.local_inflow.V_75(:,:,i) = ...
        wing.state.aero.local_inflow.V_75(:,:,i) - wing.state.external.V_Wb_dt(:,:,i) ...
        .* repmat( wing.geometry.ctrl_pt_lever(:,:,i) ./ vecnorm( wing.state.aero.local_inflow.V_25(:,:,i), 2, 1 ), 3, 1 );
end

end