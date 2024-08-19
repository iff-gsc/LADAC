function [T, Q, Zw, ZTheta] = bemResponseAndControlDerivativesNumPertub(N, Theta, V_kb, C_l_alpha, C_d_0, C_d_1, C_d_2, B, R, chord, Nb, rho)


% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Fabian Guecker
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

c           = chord;

omega       = 2 * pi * ( N / 60 );
omega_R     = omega * R;

sigma       = Nb * c / ( pi * R );
A           = pi * R^2;

Theta_Soll = Theta * 180 / pi;

% climb velocity
mu_z = -V_kb(3) / omega_R;

mu = sqrt(V_kb(1)^2 + V_kb(2)^2) / omega_R;

function_lambda = @(T) inducedVelocityWithUnits(T, V_kb, R, rho) + mu_z*omega_R;

function_handle = @(T) ( 6.0 * (T / (rho * A * omega_R ^ 2)) / ( sigma * C_l_alpha)  + 3.0 * (function_lambda(T) / omega_R) / (2.0 * B) ) * 180 / pi - (1.0 + 3/2 * mu^2)*Theta_Soll;

T = fzero(function_handle, 1);

[~, ~, v_total, ~, ~, ~] = inducedVelocityWithUnits(T, V_kb, R, rho);

lambda = v_total / omega_R;

C_T   = (T / (rho * A * omega_R ^ 2));

C_P = C_T * lambda / B + sigma * C_d_0 / 8 ...
    + sigma * C_d_1 / 2 * (Theta / 4 - lambda / ( 3 * B)) ...
    + sigma * C_d_2 / 2 * (Theta^2 / 4 - 2 * Theta * lambda / (3 * B) + lambda^2 / ( 2 * B^2 ));

Q = C_P * (rho * A * omega_R ^ 3) / omega;

T_ref = T;

%% Gradienten Berechnen ===================================================

delta_v = 1e-4;
delta_theta = 1e-6;

%% Z_w - Derivativ

V_kb_pert = V_kb + [0 0 +delta_v];
mu_z      = -V_kb_pert(3) / omega_R;

function_lambda_1 = @(T1) inducedVelocityWithUnits(T1, V_kb_pert, R, rho) + mu_z*omega_R;
function_handle_1 = @(T1) ( 6.0 * (T1 / (rho * A * omega_R ^ 2)) / ( sigma * C_l_alpha)  + 3.0 * (function_lambda_1(T1) / omega_R) / (2.0 * B) ) * 180 / pi - (1.0 + 3/2 * mu^2)*Theta_Soll;
T_pert_1 = fzero(function_handle_1, 1);

Zw = -(T_pert_1 - T_ref) / (delta_v);

%% Z_Theta - Derivativ

Theta_pert = Theta_Soll + delta_theta / 180 * pi ;
mu_z       = -V_kb(3) / omega_R;

function_lambda_2 = @(T2) inducedVelocityWithUnits(T2, V_kb, R, rho) + mu_z*omega_R;
function_handle_2 = @(T2) ( 6.0 * (T2 / (rho * A * omega_R ^ 2)) / ( sigma * C_l_alpha)  + 3.0 * (function_lambda_2(T2) / omega_R) / (2.0 * B) ) * 180 / pi - (1.0 + 3/2 * mu^2)*Theta_pert;
T_pert_2 = fzero(function_handle_2, 1);

ZTheta = -(T_pert_2-T_ref) / (delta_theta) * 180 / pi;

end
