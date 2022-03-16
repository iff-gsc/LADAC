function [ V_A_P, q_P, alpha_M_P, beta_M_P, q_Aw, M_wb ] = ...
    simpleWingGetInflow( wing, V_Ab, Omega_Ab, rho, V_Ab_prop, ...
    incidence, xyz_wing_b ) %#codegen
% simpleWingGetInflow computes the local inflow quantaties of the simple wing
% model (usually 2 point aerodynamics) depending on the aircraft state.
% 
% Inputs:
%   V_Ab            airspeed (3x1) vector in body frame, in m/s
%   cntrl_pt_wing_b concentrated control point vectors (3xn) for n control
%                   points (usually n=2) in body frame, in m
%   Omega_Ab        concentrated aerodynamic angular velocity vectors (3xn)
%                   for n control points in body frame, in rad/s
%   rho             air density (scalar), in kg/m^3
%   V_Ab_prop       concentrated propeller slidstreams (3xn) that will be
%                   added to the airspeed vector at each control point, in
%                   m/s.
%                   This should be zero if no propellers are mounted in
%                   front of the wing.
%   incidence       incidence angle (scalar) of the wing with respect to
%                   the body frame, in rad
%   xyz_wing_b      wing origin (3x1) vector represented in the body frame,
%                   in m/s
% 
% Outputs:
%   V_A_P           absolute airspeed (1xn) at each wing control point for 
%                   n control points, in m/s
%   q_P             dynamic pressure (1xn), in Pa
%   alpha_M_P       modified angle of attack (1xn) seen by each wing
%                   control point, in rad
%   beta_M_P        modified sideslip angle (1xn) seen by each wing control
%                   point, in rad
%   q_Aw            aerodynamic pitch rate (1xn) for each control point in
%                   wing frame, in rad/s
%   M_wb            concentrated rotation matrices (3x3xn) for each wing
%                   control point
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2021 Lars Killian
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% **************************************************************************

% rotate coordinate system into incidence angle
cos_i       = cos(incidence);
sin_i       = sin(incidence);
M_fw_b      = [ cos_i, 0, -sin_i; 0, 1, 0; sin_i, 0, cos_i ];

% rotate about dihedral angle
cos_v = cos(wing.geometry.v);
sin_v = sin(wing.geometry.v);

% roataion matrices from flat wing frame to each wing frame
M_w_fw_left        = [ 1 0 0; 0 cos_v sin_v; 0 -sin_v cos_v ];
M_w_fw_right       = [ 1 0 0; 0 cos_v -sin_v; 0 sin_v cos_v ];

% concentrated rotation matrices from body frame to each wing side frame
M_wb        = zeros(3,3,size(wing.xyz_cntrl_pt_wing,2));
M_wb(:,:,1) = M_w_fw_left * M_fw_b;
M_wb(:,:,2) = M_w_fw_right * M_fw_b;

% reduce roll and yaw rate to consider asymmetric downwash
% (see simpleWingSetRollRateRed and simpleWingSetYawRateRed)
Omega_Aw    = [ M_wb(:,:,1)*Omega_Ab, M_wb(:,:,2)*Omega_Ab ];
Omega_Aw    = Omega_Aw .* repmat( [wing.roll_rate_red_factor; 1; wing.yaw_rate_red_factor ], 1, 2 );
Omega_Ab_v  = [ M_wb(:,:,1)'*Omega_Aw(:,1), M_wb(:,:,2)'*Omega_Aw(:,2) ];

% compute inflow at specified points
V_Ab_P      = [ velocityFromRot( V_Ab, Omega_Ab_v(:,1), xyz_wing_b ...
            + wing.xyz_cntrl_pt_wing(:,1) ), ...
            velocityFromRot( V_Ab, Omega_Ab_v(:,2), xyz_wing_b ...
            + wing.xyz_cntrl_pt_wing(:,2) ) ] + V_Ab_prop;
argSqrt     = max( 0, V_Ab_P(1,:).^2 + V_Ab_P(2,:).^2 + ...
                V_Ab_P(3,:).^2 );
V_A_P       = real( sqrt( argSqrt ) );
q_P         = rho/2 * V_A_P.^2;

% airspeed vector in wing frame for each wing side
V_Aw_P = [ M_wb(:,:,1)*V_Ab_P(:,1), M_wb(:,:,2)*V_Ab_P(:,2) ];

% aerodynamic angles seen by each wing side
[ alpha_M_P, beta_M_P ] = aeroAnglesMod( V_Aw_P );

% compute pitch rate in wing frame with a shortcut to avoid rotation
% matrix overhead
q_Aw = [ ...
    cos_v * Omega_Ab(2) - sin_v * Omega_Ab(3), ...
    cos_v * Omega_Ab(2) + sin_v * Omega_Ab(3) ...
    ];

end
