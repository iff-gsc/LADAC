
% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

%% fit coefficients
example_airfoilAnalytic0515Al;


%% init net (NeuN (the_real_master) must be on the path)

addpath(genpath('/home/yannic/dev/FDM/neun'));
% addpath(genpath('C:\Users\Yannic\Documents\FDM\NeuN'));


%% train neural network

inputs = permute( Maq, [1,3,2] );

targets_cl = permute( fclvq./fclmax, [1,3,2] );
[weights_cl,mse_cl] = airfoilAnalytic0515NeunFit( 'netSettings_fcl', inputs, targets_cl, 1e-4 );

targets_cd = permute( fcdvq./fcdmax, [1,3,2] );
[weights_cd,mse_cd] = airfoilAnalytic0515NeunFit( 'netSettings_fcl', inputs, targets_cd, 2e-4 );

targets_cm = permute( fcmvq./fcmmax, [1,3,2] );
[weights_cm,mse_cm] = airfoilAnalytic0515NeunFit( 'netSettings_fcm', inputs, targets_cm, 5e-4 );

targets_dcl = permute( fdclvq./fdclmax, [1,3,2] );
[weights_dcl,mse_dcl] = airfoilAnalytic0515NeunFit( 'netSettings_fdc', inputs, targets_dcl, 5e-4 );

targets_dcd = permute( fdcdvq./fdcdmax, [1,3,2] );
[weights_dcd,mse_dcd] = airfoilAnalytic0515NeunFit( 'netSettings_fdc', inputs, targets_dcd, 5e-4 );

targets_dcm = permute( fdcmvq./fdcmmax, [1,3,2] );
[weights_dcm,mse_dclm] = airfoilAnalytic0515NeunFit( 'netSettings_fdc', inputs, targets_dcm, 5e-4 );


%%

Ma_eval_idx = 3;
delta_eval_idx = 1;

fcl = fclv(:,Ma_eval_idx);
fcd = fcdv(:,Ma_eval_idx);
fcm = fcmv(:,Ma_eval_idx);

[c_L_alpha_max_fit,alpha_0_fit] = airfoilAnalytic0515ClAlphaMax( fcl, Mach(Ma_eval_idx) );

figure

subplot(2,2,1)
c_L_fit = reshape( airfoilAnalytic0515AlCl( fcl, ...
    grid2Coordinates( alpha_deg, Mach(Ma_eval_idx) )' ), [], length(alpha_deg) );
plot( F15_bl.alpha.data, squeeze( map_cl(:,Ma_eval_idx,1,2,delta_eval_idx)' ), 'x' )
hold on
plot( alpha_deg, c_L_fit )
plot( alpha_deg, c_L_alpha_max_fit .* (alpha_deg-alpha_0_fit) )
analytic_airfoil = airfoilAnalytic0515LoadParams( 'airfoilAnalytic0515_params_F15' );
fcl = airfoilAnalytic0515Ma( analytic_airfoil.wcl, Mach(Ma_eval_idx), analytic_airfoil.ncl, analytic_airfoil.ocl );
c_L = reshape( airfoilAnalytic0515AlCl( fcl, ...
    grid2Coordinates( alpha_deg, Mach(Ma_eval_idx) )' ), [], length(alpha_deg) );
plot( alpha_deg, c_L )
[c_L_alpha_max,alpha_0] = airfoilAnalytic0515ClAlphaMax( fcl, Mach(Ma_eval_idx) );
plot( alpha_deg, c_L_alpha_max .* (alpha_deg-alpha_0) )
grid on
xlim([-5,15])
xlabel('alpha, deg')
ylabel('c_L, -')
legend('Given data','Curve fit','Linear curve fit','NeuN fit','Linear NeuN fit')


subplot(2,2,2)
plot( F15_bl.alpha.data, squeeze( map_cd(:,Ma_eval_idx,1,2,delta_eval_idx)' ), 'x' )
hold on
plot( alpha_deg, airfoilAnalytic0515AlCd( fcd, alpha_deg ) )
fcd = airfoilAnalytic0515Ma( analytic_airfoil.wcd, Mach(Ma_eval_idx), analytic_airfoil.ncd, analytic_airfoil.ocd );
plot( alpha_deg, airfoilAnalytic0515AlCd( fcd, alpha_deg ) )
xlim([-5,15])
xlabel('alpha, deg')
ylabel('c_D, -')
legend('Given data','Curve fit','NeuN fit')


subplot(2,2,3)
f_st_fit = airfoilDynStallFst(c_L_fit,c_L_alpha_max_fit,alpha_deg-alpha_0_fit);
plot( F15_bl.alpha.data, squeeze( map_cm(:,Ma_eval_idx,1,2,delta_eval_idx)' ), 'x' )
hold on
plot( alpha_deg, airfoilAnalyticBlCm( fcm, f_st_fit, c_L_fit ) )
fcm = airfoilAnalytic0515Ma( analytic_airfoil.wcm, Mach(Ma_eval_idx), analytic_airfoil.ncm, analytic_airfoil.ocm );
f_st = airfoilDynStallFst(c_L,c_L_alpha_max,alpha_deg-alpha_0);
plot( alpha_deg, airfoilAnalyticBlCm( fcm, f_st, c_L ) )
xlim([-5,15])
xlabel('alpha, deg')
ylabel('c_m, _')
legend('Given data','Curve fit','NeuN fit')