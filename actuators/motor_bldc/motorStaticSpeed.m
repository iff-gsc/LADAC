function omega = motorStaticSpeed( K_T, R, V_bat, d, u )

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

if d ~= 0
    omega = ( sqrt( 4*d*K_T*R*V_bat*u + K_T^4) - K_T^2 ) / ( 2*d*R );
else
    omega = V_bat/K_T*u;
end

end