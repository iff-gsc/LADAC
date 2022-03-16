function  [T, M, R_kb, Q_kb, C_T, mu, mu_z, lambda_i] = bemQuasiStaticThrustFromTheta(N, V_kb, Theta, rho, R, c, Nb, C_l_alpha, C_d_0, C_d_1, C_d_2)

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Fabian Guecker
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

T = 0;
M = 0;
R_kb = [0 0 0]';
Q_kb = [0 0 0]';


%% Pre-calculation of variables
B = 1;

N_bar = max(abs(N), 1e-8);

omega       = 2 * pi * ( N_bar / 60 );
omega_R     = omega * R;

A           = pi * R^2;
sigma       = Nb * c / ( pi * R );

mu          = sqrt(V_kb(1)^2 + V_kb(2)^2) / omega_R;
mu_z        = -V_kb(3) / omega_R;

%% Solve for steady-state thrust coefficent

function_lambda = @(CT) inducedVelocity(CT, mu, mu_z) + mu_z;
function_handle = @(CT) (6*CT / ( sigma * C_l_alpha) + 3*function_lambda(CT) / 2) - (1.0 + 3/2 * mu^2) * Theta;

% Well known to work, should be used for code generation
% C_T = Bisection_Solver(function_handle, -2e-2, 2e-2);

% Faster calculation in Simulink
C_T = fzero(function_handle, 0);

% Calculate induced flow lamda_i from C_T
lambda_i    = inducedVelocity(C_T, mu, mu_z);

% Calculate total flow lambda from lambda_i and mu_z
lambda      = lambda_i + mu_z;

%% Check if mean alpha < 15 deg, else throw an warning
alpha       = -( 3 / 2 * lambda - Theta);

if (abs(alpha) > 15 / 180 * pi)
    error('Results may be inaccurate: alpha > 15 deg, possible stall conditions ...')
end

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

