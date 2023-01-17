% This test loads airfoil data, performs an analytic fit and visualizes the
% results.

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************


data = airfoilLoadXfoilData( 'xf-e335-il-1000000' );
airfoilAnalyticBlAlFit( data.alpha, data.c_L, data.c_m, 1 );
data = airfoilLoadXfoilData( 'xf-fx78pk188-il-1000000' );
airfoilAnalyticBlAlFit( data.alpha, data.c_L, data.c_m, 1 );
data = airfoilLoadXfoilData( 'xf-goe508-il-1000000' );
airfoilAnalyticBlAlFit( data.alpha, data.c_L, data.c_m, 1 );
data = airfoilLoadXfoilData( 'xf-gs1-il-1000000' );
airfoilAnalyticBlAlFit( data.alpha, data.c_L, data.c_m, 1 );
data = airfoilLoadXfoilData( 'xf-mh200-il-1000000-n5' );
airfoilAnalyticBlAlFit( data.alpha, data.c_L, data.c_m, 1 );
data = airfoilLoadXfoilData( 'xf-n63412-il-1000000' );
airfoilAnalyticBlAlFit( data.alpha, data.c_L, data.c_m, 1 );
data = airfoilLoadXfoilData( 'xf-naca0015-il-1000000-n5' );
airfoilAnalyticBlAlFit( data.alpha, data.c_L, data.c_m, 1 );
data = airfoilLoadXfoilData( 'xf-naca66210-il-1000000' );
airfoilAnalyticBlAlFit( data.alpha, data.c_L, data.c_m, 1 );
