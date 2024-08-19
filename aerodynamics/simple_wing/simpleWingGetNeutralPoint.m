function [ xyz_np_w ] = simpleWingGetNeutralPoint( wing, alpha_M, beta_M ) %#codegen
% simpleWingGetNeutralPoint computes the current neutral points of a simple
%   wing model in the wing frame.
%   Note that the neutral point is not constant for high angles of attack
%   or high sideslip angles. This is especially important for vertical
%   takeoff and landing airplanes.
%   Indexes: w - wing frame, np - neutral point
%
% Inputs:
%   wing                simple wing struct (see simpleWingLoadParams)
%   alpha_M             modified angle of attack (1x2 array), in rad
%   betea_M             modified sideslip angle (1x2 array), in rad
% 
% Outputs:
%   xyz_np_w            concentrated position of the neutral points in wing
%                       frame (3x2 array), in m
% 
% Literature:
%   [1] Schlichting, H., & Truckenbrodt, E. A. (2013). Aerodynamik des
%       Flugzeuges: Zweiter Band: Aerodynamik des Tragfluegels (Teil II), 
%       des Rumpfes, der Fluegel-Rumpf-Anordnungen und der Leitwerke. 
%       Springer-Verlag.
% 
% See also:
%   simpleWingLoadParams, aeroAnglesMod
% 

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

xyz_np_w = wing.xyz_wing_np;
c = wing.geometry.c;
delta_y_np_a0b90 = wing.delta_y_np_a0b90_b;
delta_x_np_a0b90 = wing.delta_x_np_a0b90_b;
lambda = wing.geometry.b^2/wing.geometry.S;

% 2nd term to shift the neutral point to 3/4 in case of backward inflow;
% 3rd term to shift the neutral point backward for lateral inflow
xyz_np_w(1,:) = xyz_np_w(1,:) ...
    - c/4 * ( 1 - cos(alpha_M) .* cos(beta_M) ) ...
    + delta_x_np_a0b90 .* 0.5*(1-cos(2*beta_M)) .* cos(alpha_M);

% 2nd term to achieve high shift for beta=90deg; 3rd term to achieve
% satisfactory agreement with rolling moment due to sideslip ([1], Fig.
% 7.64)
xyz_np_w(2,:) = xyz_np_w(2,:) + ( ...
    delta_y_np_a0b90 .* 0.5*(1-cos(2*beta_M)).*cos(alpha_M).^2  ...
    + 0.5 * 0.5*(cos(2*beta_M)+1)/lambda ) ...
    .*sin(beta_M); 

end