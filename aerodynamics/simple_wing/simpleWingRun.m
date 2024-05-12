function out = simpleWingRun(wing,V_Ab,Omega_Ab,rho,V_Ab_prop,eta,incidence,pos_cg_b)
% simpleWingRun run simple wing calculation for one time step
% 
% Syntax:
%   out = simpleWingRun(wing,V_Ab,Omega_Ab,rho,V_Ab_prop,eta,incidence,pos_cg_b)
% 
% Inputs:
%   wing                Simple wing struct, see simpleWingLoadParams.m or
%                       simpleWingCreate.m
%   V_Ab                Airspeed vector (3x1 array) in body frame, in m/s
%	Omega_Ab            Angular velocity vector (3x1 array) in body frame,
%                       in rad/s
%	rho                 Air density (scalar), in kg/m^3
% 	V_Ab_prop           Propeller induced velocity (3x2 array) in body
%                       frame, in m/s
%	eta                 Flap deflection vector (1xN array for N flaps), in
%                       rad
%	incidence           Wing incidence angle (scalar), in rad
%	pos_cg_b            Position of simple wing origin measured from center
%                       of gravity in body frame (3x1 array), in m
% 
% Outputs:
%   out                 Several output quantities (struct) with the
%                       following fields:
%                       - axes.V_A_P    Airspeed at both wing sides (1x2
%                                       array), in m/s
%                       - axes.alpha_M_P Modified angle of attack at both
%                                       wing sides (1x2 array), in rad
%                       - axes.beta_M_P Sideslip angle at both wing sides
%                                       (1x2 array), in rad
%                       - dynWing.C_XYZ_a Force coefficients at both wing
%                                       sides in aerodynamic frame (3x2
%                                       array), non-dimensional
%                       - dynBody.R_Ab	Resulting force vector in body
%                                       frame (3x1 array), in N
%                       - dynBody.Q_Ab	Resulting moment vector w.r.t. the
%                                       center of gravity in body frame
%                                       (3x1 array), in Nm
% 
% See also:
%   simpleWingLoadParams, wingCreate, simpleWingGetDerivs, wingGetDerivs

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2024 Yannic Beyer
%   Copyright (C) 2024 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

[ V_A_P, q_P, alpha_M_P, beta_M_P, q_Aw, M_wb ] = ...
    simpleWingGetInflow( wing, V_Ab, Omega_Ab, rho, V_Ab_prop, ...
    incidence, pos_cg_b );

axes.V_A_P      = V_A_P;
axes.q_P        = q_P;
axes.alpha_M_P  = alpha_M_P;
axes.beta_M_P   = beta_M_P;

V_A = mean(V_A_P);

[ C_L, C_L_eta ] = simpleWingGetCl( wing, alpha_M_P, beta_M_P, eta );

C_D = simpleWingGetCd( wing, alpha_M_P, beta_M_P, eta );

C_XYZ_a = cat( 1, reshape(-C_D,1,[]), reshape(0*C_D,1,[]), reshape(-C_L,1,[]) );

M_wa = dcmBaFromAeroAnglesMod( alpha_M_P, beta_M_P );

len = length(C_XYZ_a(1,:));
C_XYZ_i_w = zeros(3,len);
for i = 1:len
    C_XYZ_i_w(:,i) = M_wa(:,:,i) * C_XYZ_a(:,i);    
end

xyz_np_w = simpleWingGetNeutralPoint( wing, alpha_M_P, beta_M_P );

x_np_wing = xyz_np_w(1,:);

C_m_camber_np_w = simpleWingGetCmCamber( wing, x_np_wing, alpha_M_P, beta_M_P );

C_lmn_flap_np_w = simpleWingGetFlapMoments( wing, C_L_eta, xyz_np_w, alpha_M_P, beta_M_P );

C_m_damp_w = simpleWingGetPitchDamping( wing, q_Aw, V_A );

C_lmn_i_w = forceCoeffs2MomentCoeffs( xyz_np_w, C_XYZ_i_w, wing.geometry.c, wing.geometry.b );

% sum of all moment coefficients in wing (w) frame
C_lmn_sum_w = C_lmn_i_w + C_lmn_flap_np_w;
C_lmn_sum_w(2,:) = C_lmn_sum_w(2,:) + C_m_camber_np_w + C_m_damp_w;

S_P = wing.geometry.S/2*[1,1];
R_Aw = zeros( size( C_XYZ_i_w ) );
Q_Aw = zeros( size( C_XYZ_i_w ) );
for i = 1:size(C_XYZ_i_w,2)
    [ R_Aw(:,i), Q_Aw(:,i) ] = coeffs2ForcesMoments( ...
        C_lmn_sum_w(:,i), C_XYZ_i_w(:,i), q_P(:,i), S_P(:,i), wing.geometry.b, wing.geometry.c );
end


dynWing.C_m_camber_np_w    = C_m_camber_np_w;
dynWing.C_lmn_i_w          = C_lmn_sum_w;
dynWing.xyz_np_w           = xyz_np_w;
dynWing.C_XYZ_i_w          = C_XYZ_i_w;
dynWing.C_XYZ_a            = C_XYZ_a;
dynWing.R_Aw               = R_Aw;
dynWing.Q_Aw               = Q_Aw;




M_bw = permute( M_wb, [2,1,3] );

len = length(C_m_camber_np_w(1,:));
C_m_camber_np_b = zeros(3,len);
for i = 1:len
    C_m_camber_np_b(:,i) = M_bw(:,2,i) * C_m_camber_np_w(:,i);    
end

xyz_np_b = zeros(size(xyz_np_w));
for i = 1:size(xyz_np_w,2)
    xyz_np_b(:,i) = xyz_np_w(:,i) + M_bw(:,:,i)*pos_cg_b;
end

len = length(C_XYZ_i_w(1,:));
C_XYZ_i_b = zeros(3,len);
for i = 1:len
    C_XYZ_i_b(:,i) = M_bw(:,:,i) * C_XYZ_i_w(:,i);    
end

R_y = zeros(size(R_Aw));
Q_y = zeros(size(R_Aw));
for i = 1:size(R_Aw,2)
    [ R_y(:,i), Q_y(:,i) ] = forceMomentTransform( R_Aw(:,i), Q_Aw(:,i), ...
        pos_cg_b, M_bw(:,:,i) );
end

R_Ab = sum(R_y,2);
Q_Ab = sum(Q_y,2);

dynBody.C_m_camber_np_b = C_m_camber_np_b;
dynBody.xyz_np_b        = xyz_np_b;
dynBody.C_XYZ_i_b       = C_XYZ_i_b;
dynBody.C_XYZ_a         = C_XYZ_a;
dynBody.R_Ab            = R_Ab;
dynBody.Q_Ab            = Q_Ab;


out.axes    = axes;
out.dynWing = dynWing;
out.dynBody = dynBody;

end
