function u = motorStaticSpeed2u( K_T, R, V_bat, d, omega )

    % Disclamer:
    %   SPDX-License-Identifier: GPL-3.0-only
    % 
    %   Copyright (C) 2022 Yannic Beyer
    %   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
    % *************************************************************************
    
    if d ~= 0
        u = ( (omega*2*d*R + K_T^2).^2 - K_T^4 ) / ( 4*d*K_T*R*V_bat );
    else
        u = K_T/V_bat*omega;
    end
    
end