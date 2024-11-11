function [alpha_htp_dalpha,alpha_htp_deta] = wingGetDownwash( vlm_wing, pos )
% wingGetDownwash computes the downwash of a wing at specified points (for
%   example at the horizontal tailplane (HTP)) using lifting line theory
% 
% Syntax:
%   [alpha_htp_dalpha,alpha_htp_deta] = wingGetDownwash( vlm_wing_main, pos )
% 
% Inputs:
%   vlm_wing            VLM wing struct (see wingCreate)
%   pos                 Specified points, where the downwash should be
%                       computed (3xN matrix for N points), in m
% 
% Outputs:
%   alpha_htp_dalpha    Induced angle of attack gradients at the specified
%                       points per angle of attack of the wing
%                       @alpha_htp/@alpha (1xN matrix), non-dimensional
%   alpha_htp_deta      Induced angle of attack gradients at the specified
%                       points per wing flap deflection angles
%                       @alpha_htp/@eta (MxN matrix for M flaps),
%                       non-dimensional
% 
% See also:
%   wingCreate, vlmVoringsLlt

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2024 Yannic Beyer
%   Copyright (C) 2024 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

num_pts = size(pos,2);

b = vlm_wing.params.b;

wing_origin = vlm_wing.geometry.origin;
AIC3 = vlmVoringsLlt( vlm_wing.geometry.vortex.pos(:,:,1) + wing_origin, ...
    vlm_wing.geometry.ctrl_pt.pos + wing_origin, 1, 10*b );
AIC = squeeze(AIC3(3,:,:));

uvw_downwash = vlmVoringsLlt( vlm_wing.geometry.vortex.pos(:,:,1) + wing_origin, pos, 1, 10*b );
AIC_downwash = squeeze(uvw_downwash(3,:,:));

num_act = vlm_wing.params.num_actuators;
alpha_htp_deta = zeros(num_act,num_pts);
act_state = zeros(1,num_act);
for i = 1:num_act
    act_state(:) = 0;
    act_state(i) = 1;
    vlm_wing = wingSetActuators( vlm_wing, act_state, zeros(1,num_act) );
    F = airfoilFlapEffectiveness(vlm_wing.geometry.segments.flap_depth);
    delta_qs = airfoilFlapDeltaQs( F.F_10, F.F_11,...
        1, vlm_wing.state.geometry.ctrl_pt.c, vlm_wing.state.actuators.segments.pos(1,:), ...
        zeros(1,vlm_wing.n_panel) ) .* cos(0);
    alpha_htp_deta(i,:) = -AIC_downwash * inv(AIC) * delta_qs(1,:)';
end
alpha_htp_dalpha = (-AIC_downwash * inv(AIC) * ones(vlm_wing.n_panel,1))';

end
