function wing = simpleWingSetYawRateRed( wing )
% simpleWingSetYawRateRed computes a factor (0~1) that compensates the
% yaw damping for a wing that has no infinite aspect ratio.
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

c_l_d_Omega_z = yawRollMoment( lambda, 2*pi, 1 );

lambda_inf = 1000;
c_l_d_Omega_z_inf = yawRollMoment( lambda_inf, 2*pi, 1 );

wing.yaw_rate_red_factor = c_l_d_Omega_z / c_l_d_Omega_z_inf;


    % [1], eq. (7.210)
    function c_l_d_Omega_x = yawRollMoment( Lambda, c_L_alpha_inf, c_L )
        k = pi*Lambda/c_L_alpha_inf;
        c_l_d_Omega_x = -0.25*(1+(sqrt(k^2+1)+1)/(sqrt(k^2+4)+2))*c_L;
    end

end
