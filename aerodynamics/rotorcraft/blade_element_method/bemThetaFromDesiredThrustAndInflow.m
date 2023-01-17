function [Theta_deg, C_T, lambda_i] = bemThetaFromDesiredThrustAndInflow(T, N, V_kb, rho, R, c, Nb, C_l_alpha)


% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Fabian Guecker
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% [1] Wall2015
% [2] Padfield

N_bar = max(abs(N), 1e-8);

omega       = 2 * pi * ( N_bar / 60 );
omega_R     = omega * R;

A           = pi * R^2;
sigma       = Nb * c / ( pi * R );

mu          = sqrt(V_kb(1)^2 + V_kb(2)^2) / omega_R;
mu_z        = -V_kb(3) / omega_R;

C_T         = T  / (rho * A * (omega_R) ^ 2);

lambda_i    = inducedVelocity(C_T, mu, mu_z);
lambda      = lambda_i + mu_z;

%% Calculation of Theta from Thrust-Coefficent CT 

% CT for untwisted blade according to [2], p.181 better approximation of
% forward flight

Theta_rad = 3*((2 * C_T  / (sigma * C_l_alpha)) + lambda/2) / (1 + 1.5 *mu^2);


%% Check if mean alpha < 15 deg, else throw an warning
alpha       = -( 3 / 2 * lambda - Theta_rad);

if (abs(alpha) > 15 / 180 * pi)
    error('Results may be inaccurate: alpha > 15 deg, possible stall conditions ...')
end

Theta_deg = Theta_rad * 180 / pi;

end
