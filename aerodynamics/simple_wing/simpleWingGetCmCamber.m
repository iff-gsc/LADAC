function [ C_m_camber_np_w ] = simpleWingGetCmCamber( wing, x_np_wing, alpha_M, beta_M ) %#codegen
% simpleWingGetCmCamber computes the pitching moment coefficient due to
%   camber with respect to the neutral point (np) in the wing (w) frame.
% 
% Inputs:
%   wing            wing struct (see simpleWingLoadParams)
%   x_wing_b        wing position (wing frame origin) in b frame (scalar),
%                   in m
%   alpha_M         modified angles of attack (1x2 vector), in rad
%   beta_M          modified sideslip angles (1x2 vector), in rad
% 
% Outputs:
%   C_m_camber_np_w pitching moment coefficient due to camber with respect
%                   to the neutral point in wing frame
% 

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

C_L0 = wing.polar.params.C_L0;
c = wing.geometry.c;
% distance from neutral point (variable) to center of pressure for
% camber (fixed, approx. located at c/2)
x_cp0_camber_wing = wing.x_cp0_camber_wing + ( x_np_wing - wing.xyz_wing_np(1,:) );
phi = wing.geometry.phi * [1,-1];

% The lift coefficient due to camber (C_L0 and C_L_eta) is applied at the
% center of pressure for camber (x_cp0_camber_b).
% C_L0 is zero for high angles of attack or high sideslip angles that is
% why it is multiplied with cos functions.
C_m_camber_np_w = C_L0 * cos(alpha_M) .* cos(beta_M+phi) .* x_cp0_camber_wing / c;

end

