% This script tests the performance (computation time) of different methods
% to compute the lift coefficient for different Mach numbers.

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

%% fit coefficients
example_airfoilAnalytic0515Al;


%%

Ma_vec = (0:0.01:0.84);

airfoil_F15 = airfoilAnalytic0515LoadParams( 'airfoilAnalytic0515_params_F15' );
microtab_F15 = airfoilMicroTabLoadParams( 'airfoilMicroTab_params_F15_90' );

tic
for i = 1:1000
    fMa = airfoilAnalytic0515Ma( airfoil_F15.wcl, Ma_vec, airfoil_F15.ncl, airfoil_F15.ocl );
end
toc

tic
for i = 1:1000
    fcl = interp1([0.15,0.7,0.78],fclv,Ma_vec,'pchip');
end
toc


Ma_vec = 0.5;
delta_vec = 0.1;

alMaDe = grid2Coordinates( alpha_deg, Ma_vec, delta_vec )';

tic
for i = 1:1000
    fMa = airfoilAnalytic0515Ma( airfoil_F15.wcl, alMaDe(2,:), airfoil_F15.ncl, airfoil_F15.ocl );
    fDe = airfoilAnalytic0515Ma( microtab_F15.net.wcl, alMaDe(2,:), microtab_F15.net.ncl, microtab_F15.net.ocl );
    [c_L_alpha_max,~] =  airfoilAnalytic0515ClAlphaMax( fMa, alMaDe(2,:) );
    [F_10,~] = airfoilFlapEffectiveness(0.25);
    c_L_matNN3 = airfoilAnalytic0515AlCl( fMa, alMaDe(1:2,:) );
    c_L_matNN3 = c_L_matNN3 ...
        + rad2deg( c_L_alpha_max ) .* F_10/pi .* alMaDe(3,:) ...
        + airfoilAnalytic0515De(fDe,alMaDe([1,3],:));
end
toc

alMaEtc = grid2Coordinates( alpha_deg, Ma_vec, 0, 0, 0 )';

aifoil_map = F15_bl;
tic
for i = 1:1000
    c_L_matLin = wingAirfoilMapInterpCoeff(aifoil_map,1,'c_L',deg2rad(alMaEtc(1,:)),alMaEtc(2,:),alMaEtc(3,:),alMaEtc(4,:),alMaEtc(5,:));
end
toc