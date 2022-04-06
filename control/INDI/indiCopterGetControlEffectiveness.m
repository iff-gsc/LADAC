function [G10,G20] = indiCopterGetControlEffectiveness( copter )
% G1 = G10 * omega * @omega/@u (see motorStaticSpeed and motorStaticSpeedDeriv)
% G2 = G20 * @omega/@u (see motorStaticSpeedDeriv)

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

[ k, d ] = propMapFitGetFactors(copter.prop.map_fit);
num_motors = size(copter.config.propPos_c,2);
L = cross( copter.config.propPos_c,repmat([0;0;-1],1,num_motors) );
M = zeros(3,num_motors);
for i=1:num_motors
    M(:,i) = -evalin('caller',['copter.config.M_b_prop',num2str(i)]) * [1;0;0];
end

G10 = 2 * inv(copter.body.I)*(k*L + d*M .* repmat(copter.config.propDir(:)',3,1));

G20 = inv(copter.body.I) * copter.prop.I * M .* repmat(copter.config.propDir(:)',3,1);

end