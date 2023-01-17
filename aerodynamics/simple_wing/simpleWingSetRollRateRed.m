function wing = simpleWingSetRollRateRed( wing )
% simpleWingSetRollRateRed computes a factor (0~1) that compensates the
% roll damping for a wing that has no infinite aspect ratio.
% 
% Literature:
%   [1] Schlichting, H. & Truckenbrodt, E. (2001): Aerodynamik des
%       Flugzeuges - Teil 2, Springer.
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

lambda = wing.geometry.b/wing.geometry.c;

c_L_d_Omega_x = rollDamping( lambda, 2*pi );

lambda_inf = 1000;
c_L_d_Omega_x_inf = rollDamping( lambda_inf, 2*pi );

wing.roll_rate_red_factor = c_L_d_Omega_x / c_L_d_Omega_x_inf;


    % [1], eq. (7.207)
    function c_L_d_Omega_x = rollDamping( Lamdbda, c_L_alpha_inf )
        k = pi*Lamdbda/c_L_alpha_inf;
        c_L_d_Omega_x = -0.25*pi*Lamdbda/(sqrt(k^2+4)+2);
    end

end
