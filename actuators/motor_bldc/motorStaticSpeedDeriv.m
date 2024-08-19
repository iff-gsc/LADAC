function omega_du = motorStaticSpeedDeriv( K_T, R, V_bat, d, u )
% d from propMapFitGetFactors

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

omega_du = u;
if d ~= 0
    omega_du(:) = divideFinite( K_T * V_bat, sqrt( K_T^4 + 4*d*K_T*R*V_bat*u ) );
else
    omega_du(:) = V_bat/K_T;
end
    
end