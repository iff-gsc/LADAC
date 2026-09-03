
% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

%% init simple wing struct
wing = simpleWingLoadParams( 'simpleWing_params_default' );

%% Example 1 (visualize C_L over alpha_M and beta_M):
alpha_M         = [-pi/2:0.05:pi/2]';
beta_M          = [-pi:0.05:pi]';
eta             = [0,0];
[ Alpha, Beta ] = meshgrid( alpha_M, beta_M );
C_L = zeros(size(Alpha));
C_L(:)          = mean(simpleWingGetCl( wing, Alpha(:)*[1,1], Beta(:)*[1,1], eta ),2);
figure
surf( rad2deg(Alpha), rad2deg(Beta), C_L )
xlabel('\alpha_M, deg'), ylabel('\beta_M, deg'), zlabel('C_L')

%% Example 2 (visualize influence of eta in C_L over alpha_M curve):
alpha_M         = [-pi/2:0.01:pi/2]';
beta_M          = 0;
eta             = [-10,0,10]'*pi/180;
figure
for i = 1:length(eta)
  C_L = simpleWingGetCl( wing, alpha_M(:)*[1,1], beta_M*[1,1], eta(i)*[1,1] );
  plot( rad2deg(alpha_M), C_L )
  hold on
end
xlabel('\alpha_M, deg'), ylabel('C_L'), grid on;

%% Example 3 (show C_L, C_D and polar for full angle of attack range)
alpha   = -linspace(-pi,pi,500)';
alpha_M = alpha;
beta_M  = zeros(size(alpha_M)) + 0;
beta_M(alpha>pi/2|alpha<-pi/2)  = pi - beta_M(alpha>pi/2|alpha<-pi/2);
alpha_M(alpha>pi/2)             = pi - alpha_M(alpha>pi/2);
alpha_M(alpha<-pi/2)            = -pi - alpha_M(alpha<-pi/2);
eta     = [-15,0,15]'*pi/180;
C_L     = zeros(size(alpha));
C_D     = zeros(size(alpha));
f1 = figure; hold on; grid on; box on;
xlabel('\alpha (deg)'); ylabel('C_L'); legend();
f2 = figure; hold on; grid on; box on;
xlabel('\alpha (deg)'); ylabel('C_D'); legend('location','north');
f3 = figure; hold on; grid on; box on;
xlabel('C_D'); ylabel('C_L'); legend();
for i = 1:length(eta)
    C_L_M = simpleWingGetCl( wing, alpha_M*[1,1], beta_M*[1,1], eta(i)*[1,1] );
    C_D_M = simpleWingGetCd( wing, alpha_M*[1,1], beta_M*[1,1], eta(i)*[1,1] );
    for j = 1:length(alpha_M)
        M_ba_M = dcmBaFromAeroAnglesMod(alpha_M(j),beta_M(j));
        C_XYZ_b = M_ba_M * [-C_D_M(j,1)';0;-C_L_M(j,1)'];
        M_ba = dcmBaFromAeroAngles(alpha(j),0);
        C_XYZ_a = M_ba'*C_XYZ_b;
        C_L(j) = -C_XYZ_a(3);
        C_D(j) = -C_XYZ_a(1);
    end
    figure(f1)
    plot( alpha*180/pi, C_L(:,1), DisplayName=['\eta = ',num2str(eta(i)*180/pi),' deg'] )
    figure(f2);
    plot( alpha*180/pi, C_D_M(:,1), DisplayName=['\eta = ',num2str(eta(i)*180/pi),' deg'])
    figure(f3);
    plot( C_D, C_L, DisplayName=['\eta = ',num2str(eta(i)*180/pi),' deg'] )
end

%% Example 4 (compute lift coefficients for two halfs of a swept wing like
%           a two point model):
alpha_M         = [ 0.11, 0.1 ];
beta_M          = [ 0.2, 0.15 ];
eta             = [ -0.1, 0.05 ];
C_L = simpleWingGetCl( wing, alpha_M, beta_M, eta )
