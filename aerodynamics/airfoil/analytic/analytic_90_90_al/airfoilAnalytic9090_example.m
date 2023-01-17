% This test loads airfoil data, performs an analytic fit and visualizes the
% results.

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

%% NACA0015 (symmetric) from -90deg to 90deg

NACA0015 = NACA0015();

alpha_deg = -90:0.1:90;
delta = 0:0.2:2;
Mach = [0.15,0.7,0.78];

[fcl,fcd] = airfoilAnalytic9090AlFit( NACA0015.alpha, NACA0015.cl, NACA0015.cd, 0 );
c_L = airfoilAnalytic9090AlCl( fcl, alpha_deg );
c_D = airfoilAnalytic9090AlCd( fcd, alpha_deg );

% plot
figure
subplot(2,1,1)
plot(NACA0015.alpha,NACA0015.cl,'x')
hold on
plot(alpha_deg,c_L)
grid on
xlim([-90,90])
xlabel('alpha, deg')
ylabel('c_L, -')
legend('data','analytic function','location','southeast')

subplot(2,1,2)
plot(NACA0015.alpha,NACA0015.cd,'x')
hold on
plot(alpha_deg,c_D)
grid on
xlim([-90,90])
xlabel('alpha, deg')
ylabel('c_D, -')
legend('data','analytic function','location','southeast')


%% additional tests from xfoil datadata = airfoilLoadXfoilData( 'xf-e335-il-1000000' );

data = airfoilLoadXfoilData( 'xf-e335-il-1000000' );
airfoilAnalytic9090AlFit( data.alpha, data.c_L, data.c_D, 1 );
data = airfoilLoadXfoilData( 'xf-fx78pk188-il-1000000' );
airfoilAnalytic9090AlFit( data.alpha, data.c_L, data.c_D, 1 );
data = airfoilLoadXfoilData( 'xf-goe508-il-1000000' );
airfoilAnalytic9090AlFit( data.alpha, data.c_L, data.c_D, 1 );
data = airfoilLoadXfoilData( 'xf-gs1-il-1000000' );
airfoilAnalytic9090AlFit( data.alpha, data.c_L, data.c_D, 1 );
data = airfoilLoadXfoilData( 'xf-mh200-il-1000000-n5' );
airfoilAnalytic9090AlFit( data.alpha, data.c_L, data.c_D, 1 );
data = airfoilLoadXfoilData( 'xf-n63412-il-1000000' );
airfoilAnalytic9090AlFit( data.alpha, data.c_L, data.c_D, 1 );
data = airfoilLoadXfoilData( 'xf-naca0015-il-1000000-n5' );
airfoilAnalytic9090AlFit( data.alpha, data.c_L, data.c_D, 1 );
data = airfoilLoadXfoilData( 'xf-naca66210-il-1000000' );
airfoilAnalytic9090AlFit( data.alpha, data.c_L, data.c_D, 1 );

