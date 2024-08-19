function C_lmn_i_b = simpleWingLocal2GlobalMomentCoeffs( wing, C_lmn_i_w, ...
    C_XYZ_i_w, M_bw, xyz_wing_b ) %#codegen
% simpleWingLocal2GlobalMomentCoeffs computes the moment coefficients
%   w.r.t. the center of gravity from force and moment coefficient w.r.t. a
%   neutral point of a simple wing.
%   The conversion requires to multiply the force coefficients that are
%   applied at the local wing neutral point with the dimensionless lever
%   arm to the center of gravity.
% 
% Inputs:
%   wing            simple wing struct (see simpleWingLoadParams)
%   C_lmn_i_w       concentrated moment coefficient vectors (3xn) w.r.t. 
%                   the neutral points for n control points in wing frame
%   M_bw            rotation matrices (3x3xn) from wing frame to body frame
%   xyz_wing_b      position (3x1) vector of the wing origin in body frame,
%                   in m
% 
% Outputs:
%   C_lmn_i_b       concentrated moment coefficient vectors (3xn) w.r.t.
%                   center of gravity for n control points in the body
%                   frame
% 

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

numCntrlPnts = size(wing.xyz_cntrl_pt_wing,2);

C_lmn_i_b = zeros(3,numCntrlPnts);

% position vector from wing frame origins to center of gravity in b frame
xyz_wing_cg_b = -xyz_wing_b;

for i = 1:numCntrlPnts
    xyz_wing_cg_w = M_bw(:,:,i)' * xyz_wing_cg_b;
    % moment coefficients due to force coefficients and lever arms
    C_lmn_i_b(:,i) = M_bw(:,:,i) * C_lmn_i_w(:,i) ...
        + M_bw(:,:,i) * ...
        forceCoeffs2MomentCoeffs( xyz_wing_b, C_XYZ_i_w(:,i), ...
        wing.geometry.c, wing.geometry.b );
end