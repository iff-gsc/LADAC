function [ C_lmn_flap_np_w ] = simpleWingGetFlapMoments( wing, C_L_eta, xyz_np_w, alpha_M, beta_M ) %#codegen
% simpleWingGetFlapMoments computes the flap moment coefficients with
%   respect to the neutral points (np) in the wing (w) frame.
% 
% Inputs:
%   wing            wing struct (see simpleWingLoadParams)
%   C_L_eta         lift coefficient due to flap deflection (1x2 vector)
%   xyz_np_w        neutral points positions in b frame (3x2 vector), in m
% 
% Outputs:
%   C_lmn_flap_np_w flap moment coefficients with respect to the neutral
%                   points in wing frame
% 

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2024 Yannic Beyer
%   Copyright (C) 2024 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% The lift coefficient due to flap deflection is applied at the points
% x_cp0_flap_wing, y_cp. Since the lift coefficient is already applied to
% the neutral points, the moments must be computed with respect to these
% points.

Delta_x_flap = wing.flap.x_cp0_wing;
Delta_x_flap(1:end/2) = Delta_x_flap(1:end/2) * cos(alpha_M(1)) * cos(beta_M(1)) - xyz_np_w(1,1);
Delta_x_flap(end/2+1:end) = Delta_x_flap(end/2+1:end) * cos(alpha_M(2)) * cos(beta_M(2)) - xyz_np_w(1,2);

Delta_y_flap = wing.flap.y_cp_wing;
Delta_y_flap(1:end/2) = Delta_y_flap(1:end/2) - xyz_np_w(2,1);
Delta_y_flap(end/2+1:end) = Delta_y_flap(end/2+1:end) - xyz_np_w(2,2);

C_m_flap_np_w_all = C_L_eta .* Delta_x_flap / wing.geometry.c;
C_m_flap_left = sum(C_m_flap_np_w_all(1:end/2));
C_m_flap_right = sum(C_m_flap_np_w_all(end/2+1:end));

C_l_flap_all = -C_L_eta .* Delta_y_flap / (wing.geometry.b/2);
C_l_flap_left = sum(C_l_flap_all(1:end/2));
C_l_flap_right = sum(C_l_flap_all(end/2+1:end));

C_lmn_flap_np_w = [ ...
    C_l_flap_left,C_l_flap_right; ...
    C_m_flap_left,C_m_flap_right; ...
    0, 0 ...
    ];

end

