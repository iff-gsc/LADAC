function [k,d] = propMapFitGetFactors( prop_map_fit )

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

V = 0;
RPM = linspace(0,prop_map_fit.boarders.RPM_max/2,20);
omega = RPM/60*2*pi;
thrust_curve = propMapFitGetZ(prop_map_fit,RPM,V,'thrust');
torque_curve = propMapFitGetZ(prop_map_fit,RPM,V,'torque');

k = fitFactor( omega, thrust_curve );
d = fitFactor( omega, torque_curve );

end

function factor = fitFactor( x, y2 )

x_mean = mean(x);
y = sqrt(abs(y2));
y_mean = mean(y);

slope = sum((x-x_mean).*(y-y_mean)) / sum((x-x_mean).^2);

factor = slope^2;

end