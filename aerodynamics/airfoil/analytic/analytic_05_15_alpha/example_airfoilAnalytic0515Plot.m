% This script visualizes the aerodynamic coefficients of different methods
% for different Mach numbers.

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

%% fit coefficients
fit_airfoilAnalytic0515Al_F15;


%%

alpha_deg = (-5:0.5:15)';
Ma_vec = (0.14:0.01:0.75)';
phi = deg2rad(17);
c_L_mat = zeros(length(alpha_deg),length(Ma_vec));
c_D_mat = zeros(length(alpha_deg),length(Ma_vec));
c_m_mat = zeros(length(alpha_deg),length(Ma_vec));

for i = 1:length(Ma_vec)
    Ma = Ma_vec(i);
    fcl = interp1(Mad,fclv,Ma,'pchip');
    fcd = interp1(Mad,fcdv,Ma,'pchip');
    fcm = interp1(Mad,fcmv,Ma,'pchip');
    
    c_L_mat(:,i) = airfoilAnalytic0515AlCl( fcl, grid2Coordinates( alpha_deg, Ma ) );
    c_D_mat(:,i) = airfoilAnalytic0515AlCd( fcd, alpha_deg );
    
    [c_L_alpha_max,alpha_0] = airfoilAnalytic0515ClAlphaMax( fcl, Ma );
    fst = airfoilDynStallFst( c_L_mat(:,i), c_L_alpha_max,alpha_deg-alpha_0 );
    c_m_mat(:,i) = airfoilAnalyticBlCm( fcm, fst, c_L_mat(:,i));
    
end


% load example parameters for F15 airfoil
airfoil_F15 = airfoilAnalytic0515LoadParams( 'airfoilAnalytic0515_params_F15' );

alMa = grid2Coordinates( alpha_deg, Ma_vec );

fclMa = airfoilAnalytic0515Ma( airfoil_F15.wcl, alMa(:,2), airfoil_F15.ncl, airfoil_F15.ocl );
fcdMa = airfoilAnalytic0515Ma( airfoil_F15.wcd, alMa(:,2), airfoil_F15.ncd, airfoil_F15.ocd );
fcmMa = airfoilAnalytic0515Ma( airfoil_F15.wcm, alMa(:,2), airfoil_F15.ncm, airfoil_F15.ocm );


c_L_matNN = airfoilAnalytic0515AlCl( fclMa, alMa );
c_D_matNN = airfoilAnalytic0515AlCd( fcdMa, alMa(:,1) );

[c_L_alpha_max,alpha_0] = airfoilAnalytic0515ClAlphaMax( fclMa, alMa(:,2) );
fst = airfoilDynStallFst( c_L_matNN, c_L_alpha_max,alMa(:,1)-alpha_0 );
c_m_matNN = airfoilAnalyticBlCm( fcmMa, fst, c_L_matNN );



alMaEtc = grid2Coordinates( alpha_deg, Ma_vec, 0, 0, 0 );

airfoil_map = F15_bl;

c_L_matLin = wingAirfoilMapInterpCoeff(airfoil_map,1,'c_L',deg2rad(alMaEtc(:,1)'),alMaEtc(:,2)',alMaEtc(:,3)',alMaEtc(:,4)',alMaEtc(:,5)');


%% plot results

figure
subplot(2,2,1)
[X,Y] = meshgrid( alpha_deg, Ma_vec );
surf( X, Y, c_L_mat', 'FaceAlpha', 0.5 )
hold on
XY = grid2Coordinates(alpha_deg,Ma_vec);
% plot3(XY(:,1),XY(:,2),c_L_matNN(:),'ro')
% plot3(XY(:,1),XY(:,2),c_L_matNN2(:),'bx')
plot3(XY(:,1),XY(:,2),c_L_matNN(:),'gs')
plot3(XY(:,1),XY(:,2),c_L_matLin(:),'ro')
xlabel('angle of attack, deg')
ylabel('Mach number')
zlabel('lift coefficient')
legend('interp1','neural net','linear map')

subplot(2,2,2)
surf( X, Y, c_D_mat', 'FaceAlpha', 0.5 )
hold on
plot3(XY(:,1),XY(:,2),c_D_matNN(:),'gs')
xlabel('angle of attack, deg')
ylabel('Mach number')
zlabel('drag coefficient')


subplot(2,2,3)
surf( X, Y, c_m_mat', 'FaceAlpha', 0.5 )
hold on
plot3(XY(:,1),XY(:,2),c_m_matNN(:),'gs')
xlabel('angle of attack, deg')
ylabel('Mach number')
zlabel('pitching moment coefficient')

