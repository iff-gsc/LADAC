function [T, M, R_kb, Q_kb, C_T, mu, mu_z] = bemThrustFromInflowParameters(N, V_kb, lambda_i, Theta, rho, R, c, Nb, C_l_alpha, C_d_0, C_d_1, C_d_2)

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Fabian Guecker
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% [1] Wall2015
% [2] Padfield

B = 1;

N_bar = max(abs(N), 1e-8);

omega       = 2 * pi * ( N_bar / 60 );
omega_R     = omega * R;

A           = pi * R^2;
sigma       = Nb * c / ( pi * R );

mu          = sqrt(V_kb(1)^2 + V_kb(2)^2) / omega_R;
mu_z        = -V_kb(3) / omega_R;

lambda      = lambda_i + mu_z;

%% Calculation of Thrust-Coefficent CT 

% CT for untwisted blade according to [1], p.181 
% C_T          = sigma * C_l_alpha / 2 * ( Theta / 3 - lambda / 2) ;

% CT for untwisted blade according to [2], p.181 better approximation of
% forward flight
C_T          = sigma * C_l_alpha / 2 * ( 1/3*(1 + 1.5 *mu^2) * Theta - lambda / 2) ;


%% Check if mean alpha < 15 deg, else throw an warning
alpha       = -( 3 / 2 * lambda - Theta);

% if (abs(alpha) > 15 / 180 * pi)
%     error('Results may be inaccurate: alpha > 15 deg, possible stall conditions ...')
% end


%% Calculation of Power-Coefficent CT 

% [1], p.181
C_P = C_T * lambda / B + sigma * C_d_0 / 8 ...
      + sigma * C_d_1 / 2 * (Theta / 4 - lambda / ( 3 * B)) ...
      + sigma * C_d_2 / 2 * (Theta^2 / 4 - 2 * Theta * lambda / (3 * B) + lambda^2 / ( 2 * B^2 ));

  
  
%% Calculation of Output Variables

rho_A_omega_R_2 = rho * A * (omega_R) ^ 2;

T = C_T * rho_A_omega_R_2;
M = C_P * rho_A_omega_R_2 * R * sign(N);

R_kb = [0, 0, -T]';
Q_kb = [0, 0, -M]';


end
