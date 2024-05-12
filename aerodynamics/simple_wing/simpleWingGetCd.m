function C_D = simpleWingGetCd( wing, alpha_M, beta_M, eta ) %#codegen
% simpleWingGetCd computes the drag coefficient of a wing using a simple
%   generic model. The model considers a characteristic curve of the drag 
%   coefficient over the angle of attack for the unswept wing, the modified
%   aerodynamic angles, a flap deflection angle, the sweep angle as well 
%   as three parameters. This function is usually used to compute the drag
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
%   C_D             drag coefficient (same dimension as alpha_M or beta_M),
%                   in 1
% 
% Examples:
% Define parameters:
%     % define simple wing characteristic (default)
%     wing = simpleWingLoadParams('params_aero_simple_wing_default')
% 
% Example 1 (visualize C_D over alpha_M and beta_M):
%     alpha_M         = -pi/2:0.05:pi/2;
%     beta_M          = -pi:0.05:pi;
%     eta             = [0,0];
%     [ Alpha, Beta ] = meshgrid( alpha_M, beta_M );
%     C_D = zeros(size(Alpha));
%     C_D(:)             = mean(simpleWingGetCd( wing, Alpha(:)*[1,1], Beta(:)*[1,1], eta ),2);
%     figure
%     surf( Alpha, Beta, C_D )
%     xlabel('\alpha_M, rad'), ylabel('\beta_M, rad'), zlabel('C_D')
% 
% Example 2 (visualize influence of eta in C_D over alpha_M curve):
%     alpha_M         = [-pi/2:0.05:pi/2]';
%     beta_M          = 0;
%     eta             = [-10,0,10]'*pi/180;
%     figure
%     for i = 1:length(eta)
%         C_D = simpleWingGetCd( wing, alpha_M*[1,1], beta_M*[1,1], eta(i)*[1,1] );
%         plot( alpha_M, C_D )
%         hold on
%     end
%     xlabel('\alpha_M, rad'), ylabel('C_D'), grid on;
% 
% Example 3 (compute drag coefficients for two halfs of a swept wing like
%           a two point model):
%     alpha_M         = [ 0.11, 0.1 ];
%     beta_M          = [ 0.2, 0.15 ];
%     eta             = [ -0.1, 0.05 ];
%     C_D = simpleWingGetCd( wing, alpha_M, beta_M, eta )
% 
% See also: aeroAnglesMod, simpleWingGetCl

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

    phi = wing.geometry.phi * [1,-1];
    C_D0 = wing.polar.params.C_D0;
    C_Dmax = wing.polar.params.C_Dmax;
    C_D_dEta = wing.flap.C_D_dEta;
    alpha_M_vector = wing.polar.alpha;
    C_D_vsAlphaVec = wing.polar.C_D;

    C_D = zeros( size( alpha_M ) );
    
    % compute the effective flap angle, but only if field exists
    if wing.flap.is_eff_scaling
        eta = simpleWingGetEffFlapAng(wing, eta);
    end
    
    % compute the effective sideslip angle depending on the sweep angle
    beta_M_eff = beta_M + phi;
    
    % compute the drag coefficient depending on alpha_M for beta_M = 0
    C_D_beta0 = interp1( alpha_M_vector, C_D_vsAlphaVec, alpha_M );
    
    % add the drag coefficient depending on a flap deflection angle eta
    C_D_eta_all = C_D_dEta * eta;
    C_D_eta_left = sum(C_D_eta_all(1:end/2));
    C_D_eta_right = sum(C_D_eta_all(end/2+1:end));
    C_D_beta0_eta = C_D_beta0 + [C_D_eta_left,C_D_eta_right] .* cos(alpha_M).^2;
    
    % compute the drag coefficient depending on alpha_M for beta_M = 90 deg
    C_D_beta90 = 0.5 * ( C_Dmax - C_D0 ) .* ...
        ( 1 - cos(2*alpha_M) ) + C_D0;
    
    % compute the final drag coefficient depending on alpha_M, beta_M and
    % eta
    C_D(1:end,1:end) = C_D_beta0_eta .* cos(beta_M_eff).^2 ...
        + C_D_beta90 .* sin(beta_M_eff).^2;
    
end