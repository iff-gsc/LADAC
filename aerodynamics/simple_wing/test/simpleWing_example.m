
% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% init simple wing struct
wing = simpleWingLoadParams( 'params_aero_simple_wing_default' );

% Example 1 (visualize C_L over alpha_M and beta_M):
alpha_M         = [-pi/2:0.05:pi/2]';
beta_M          = [-pi:0.05:pi]';
eta             = [0,0];
[ Alpha, Beta ] = meshgrid( alpha_M, beta_M );
C_L = zeros(size(Alpha));
C_L(:)          = mean(simpleWingGetCl( wing, Alpha(:)*[1,1], Beta(:)*[1,1], eta ),2);
figure
surf( Alpha, Beta, C_L )
xlabel('\alpha_M'), ylabel('\beta_M'), zlabel('C_L')

% Example 2 (visualize influence of eta in C_L over alpha_M curve):
alpha_M         = [-pi/2:0.01:pi/2]';
beta_M          = 0;
eta             = [-10,0,10]'*pi/180;
figure
for i = 1:length(eta)
  C_L = simpleWingGetCl( wing, alpha_M(:)*[1,1], beta_M*[1,1], eta(i)*[1,1] );
  plot( alpha_M, C_L )
  hold on
end
xlabel('\alpha_M'), ylabel('C_L'), grid on;

% Example 3 (compute lift coefficients for two halfs of a swept wing like
%           a two point model):
alpha_M         = [ 0.11, 0.1 ];
beta_M          = [ 0.2, 0.15 ];
eta             = [ -0.1, 0.05 ];
C_L = simpleWingGetCl( wing, alpha_M, beta_M, eta )
