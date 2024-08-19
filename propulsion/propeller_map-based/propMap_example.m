% This example script calls functions of the map-based propeller to show
% how they work.

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% load propeller map for specified propeller
prop_name = '9x9';
prop_fit = propMapFitCreate( prop_name );
prop_scatter = propMapScatterCreate( prop_name );
prop_grid = propMapGridCreate( prop_name );

% plot propeller map
figure
subplot(2,2,1)
propMapFitPlot( prop_fit, 'thrust' );
hold on
propMapScatterPlot( prop_scatter, 'thrust' );
subplot(2,2,2)
propMapFitPlot( prop_fit, 'torque' );
hold on
propMapScatterPlot( prop_scatter, 'torque' );
subplot(2,2,3)
propMapGridPlot( prop_grid, 'thrust' );
subplot(2,2,4)
propMapGridPlot( prop_grid, 'torque' );


% get list of available propellers
name_list = propMapGetNameList();

% You can also create propeller maps for propellers that are not inside the
% APC Propeller database. However, this feature should be used with
% caution (see propMapMatch).
prop_fit = propMapFitCreate( '30x10.5' );
