function prop_fit = propMapFitCreate( DATA_APC, prop_name, is_plot )
% PROPMAPFITCREATE create polynomial fit from propeller map data
%   The polynomial fit is of order 5 depending on both airspeed and
%   rotational speed.
% 
% Syntax:
%   prop_fit = propMapFitCreate( DATA_APC, prop_name )
%   prop_fit = propMapFitCreate( DATA_APC, prop_name, is_plot )
% 
% Inputs:
% 	 DATA_APC               An array of cell array which contains propeller
%                           maps from APC for several propeller types.
%                           This can be load from a mat file:
%                           load('DATA_APC');
%    prop_name              The name of one specific propeller type within
%                           the first column of DATA_APC.
%                           Use the following command to get all available
%                           names:
%                           name_list = propMapGetNameList(DATA_APC);
%   is_plot                 (optional) show curve fit result or not (bool),
%                           default is false.
% 
% Outputs:
%   prop_fit                propeller map fit struct as defined by this
%                           function (struct)
% 
% See also:
%   PROPMAPFITPLOT, PROPMAPFITGETZ, PROPMAPFITGETZDERIV

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

if nargin < 3
    is_plot = false;
end

% get scattered propeller map from cell array
prop_map_scatter = propMapScatterCreate( DATA_APC, prop_name );

% fit thrust
[fitresult, ~] = propMapCurveFit( prop_map_scatter.RPM, ...
                    prop_map_scatter.V, prop_map_scatter.thrust, ...
                    'thrust', is_plot );
coeffs = fitresult2coeffs( fitresult );
prop_fit.coeffs_thrust = coeffs;

% fit torque
[fitresult, ~] = propMapCurveFit( prop_map_scatter.RPM, ...
                    prop_map_scatter.V, prop_map_scatter.torque, ...
                    'torque', is_plot );
coeffs = fitresult2coeffs( fitresult );
prop_fit.coeffs_torque = coeffs;

prop_fit.boarders = propMapGetBoarders( prop_map_scatter );

end

function coeffs = fitresult2coeffs( fitresult )

% force zero thrust and torque at RPM=0 and V=0
coeffs.p00 = 0;
% assign other coeffs to struct
coeffs.p10 = fitresult.p10;
coeffs.p01 = fitresult.p01;
coeffs.p20 = fitresult.p20;
coeffs.p11 = fitresult.p11;
coeffs.p02 = fitresult.p02;
coeffs.p30 = fitresult.p30;
coeffs.p21 = fitresult.p21;
coeffs.p12 = fitresult.p12;
coeffs.p03 = fitresult.p03;
coeffs.p40 = fitresult.p40;
coeffs.p31 = fitresult.p31;
coeffs.p22 = fitresult.p22;
coeffs.p13 = fitresult.p13;
coeffs.p04 = fitresult.p04;
coeffs.p50 = fitresult.p50;
coeffs.p41 = fitresult.p41;
coeffs.p32 = fitresult.p32;
coeffs.p23 = fitresult.p23;
coeffs.p14 = fitresult.p14;
coeffs.p05 = fitresult.p05;

end

function [fitresult, gof] = propMapCurveFit( RPM, V, Z, output_name, is_plot )

warning('off', 'curvefit:prepareFittingData:removingNaNAndInf');
warning('off', 'curvefit:fit:equationBadlyConditioned');

RPM_ext = [ RPM, zeros( 1, length(RPM) ) ];
V_ext   = [ V, zeros( 1, length(V) ) ];
Z_ext   = [ Z, zeros( 1, length(Z) ) ];
[xData, yData, zData] = prepareSurfaceData( RPM_ext, V_ext, Z_ext );

% Set up fittype and options.
ft = fittype( 'poly55' );
opts = fitoptions( 'Method', 'LinearLeastSquares' );
opts.Robust = 'LAR';

% Fit model to data.

[fitresult, gof] = fit( [xData, yData], zData, ft );

warning('on', 'curvefit:prepareFittingData:removingNaNAndInf');
warning('on', 'curvefit:fit:equationBadlyConditioned');

if is_plot
    % Plot fit with data.
    figure;
    h = plot( fitresult, [xData, yData], zData );
    % Label axes
    xlabel('Rotational speed, rpm')
    ylabel('Airspeed, m/s')
    zlabel(output_name)
    grid on
    view( -65.8, 23.6 );
    z_max = max(zData);
    z_min = min(zData);
    Delta_z = abs(z_max-z_min);
    zlim([ z_min - 0.2*Delta_z, z_max + 0.2*Delta_z ])
end

end
