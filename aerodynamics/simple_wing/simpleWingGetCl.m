function [ C_L, C_L_eta ] = simpleWingGetCl( wing, alpha_M, beta_M, eta ) %#codegen
% simpleWingGetCl computes the lift coefficient of a wing using a simple
%   generic model. The model considers a characteristic curve of the lift 
%   coefficient over the angle of attack for the unswept wing, the modified
%   aerodynamic angles, a flap deflection angle, the sweep angle as well 
%   as three parameters. This function is usually used to compute the lift
%   coefficient of two wing halfs using a two point model. The model is 
%   documented in [1, pp. 22] and it is based on measured data presented in 
%   [2, pp. 72].
% 
% Literature:
%   [1] Beyer, Y. (2017): Flight Control Design and Simulation of a Tandem
%       Tilt Wing RPAS, master thesis, TU Braunschweig.
%   [2] Schlichting, H. & Truckenbrodt, E. (2001): Aerodynamik des
%       Flugzeuges - Teil 2, Springer.
% 
% Inputs:
%   wing            simple wing struct (see loadSimpleWingParams)
%   alpha_M         modified angle of attack (scalar, vector or matrix), in
%                   rad
%   beta_M          modified sideslip angle (scalar, same dimension as 
%                   alpha_M or vector/matrix if alpha_M is a scalar), in 
%                   rad
%   eta             flap deflection angle (scalar or same dimension as
%                   beta_M), in rad
% 
% Outputs:
%   C_L             lift coefficient (same dimension as alpha_M or beta_M),
%                   in 1
% 
% Examples:
% Define parameters:
%   % define simple wing characteristic (default)
%   wing = simpleWingLoadParams('params_aero_simple_wing_default')
% 
% Example 1 (visualize C_L over alpha_M and beta_M):
%     alpha_M         = [-pi/2:0.05:pi/2]';
%     beta_M          = [-pi:0.05:pi]';
%     eta             = [0,0];
%     [ Alpha, Beta ] = meshgrid( alpha_M, beta_M );
%     C_L = zeros(size(Alpha));
%     C_L(:)          = mean(simpleWingGetCl( wing, Alpha(:)*[1,1], Beta(:)*[1,1], eta ),2);
%     figure
%     surf( Alpha, Beta, C_L )
%     xlabel('\alpha_M'), ylabel('\beta_M'), zlabel('C_L')
% 
% Example 2 (visualize influence of eta in C_L over alpha_M curve):
%     alpha_M         = [-pi/2:0.01:pi/2]';
%     beta_M          = 0;
%     eta             = [-10,0,10]'*pi/180;
%     figure
%     for i = 1:length(eta)
%       C_L = simpleWingGetCl( wing, alpha_M(:)*[1,1], beta_M*[1,1], eta(i)*[1,1] );
%       plot( alpha_M, C_L )
%       hold on
%     end
%     xlabel('\alpha_M'), ylabel('C_L'), grid on;
% 
% Example 3 (compute lift coefficients for two halfs of a swept wing like
%           a two point model):
%     alpha_M         = [ 0.11, 0.1 ];
%     beta_M          = [ 0.2, 0.15 ];
%     eta             = [ -0.1, 0.05 ];
%     C_L = simpleWingGetCl( wing, alpha_M, beta_M, eta )
% 
% See also: aeroAnglesMod, simpleWingGetCd

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************
    
    phi = wing.geometry.phi * [1,-1];
    C_L_dEta = wing.flap.dalpha_deta * wing.polar.params.C_Lalpha;
    alpha_M_vector = wing.polar.alpha;
    C_L_vsAlphaVec = wing.polar.C_L;
    C_L_dAlp_b90a0 = wing.lateral_polar.c_Lbeta90alpha0;
    C_L_beta90max = wing.lateral_polar.c_Lbeta90max;

    C_L = zeros( size( alpha_M ) );
    
    % compute the effective flap angle, but only if field exists
    if wing.flap.is_eff_scaling
        eta = simpleWingGetEffFlapAng(wing, eta);
    end
    
    % compute the effective sideslip angle depending on the sweep angle
    beta_M_eff = beta_M + phi;
    
    % change the angle of attack for high angles of attack to shift the zero
    % crossing of the lift coefficient at 90deg angle of attack
    Delta_alpha_flap = wing.flap.dalpha_deta * eta  .* sin(alpha_M);
    
    % The polar is only defined for +-90deg angle of attack. If the flow
    % comes from behind, take the same polar (but sign must be reversed)
    alpha_M_interp = alpha_M;
    alpha_M_interp(abs(beta_M)>pi/2) = -alpha_M_interp(abs(beta_M)>pi/2);

    % compute the lift coefficient depending on alpha_M for beta_M = 0
    C_L_beta0 = interp1( alpha_M_vector, C_L_vsAlphaVec, alpha_M_interp + Delta_alpha_flap,'linear','extrap' ) ...
        .* sign( -abs(beta_M) + pi/2 );
    
    C_L_eta = C_L_dEta * eta .* cos(alpha_M) .* cos(beta_M);
        
    % add the lift coefficient depending on a flap deflection angle eta
    C_L_alpha = C_L_beta0 .* abs(cos(beta_M_eff));
    
    % compute the lift coefficient depending on alpha_M for beta_M = 90 deg
	C_L_beta90 = 0.5 * C_L_dAlp_b90a0 * sin(2*alpha_M) + ...
        0.5 * ( C_L_beta90max - 0.5 * C_L_dAlp_b90a0 ) ...
        .* sign(alpha_M) .* ( 1 - cos(4*alpha_M) );
    
    % compute the final lift coefficient depending on alpha_M, beta_M and
    % eta
    C_L(1:end,1:end) = C_L_alpha + C_L_eta ...
        + C_L_beta90 .* sin(beta_M_eff).^2;
    
end