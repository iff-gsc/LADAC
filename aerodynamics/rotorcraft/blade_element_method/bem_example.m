% Example script for evaluating the blade-element-method in LADAC

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Fabian Guecker
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% Clear workspace and console
clear all;
clc;

%% Configuration

% rotational speed RPM [U/min]
N = 2000;

% Desired thrust [N]
T = 20 *9.81 * 100 / 100;

% velocity vector of rotorhub [m/s]
V_kb     = [0 0 0]; % hover

%V_kb     = [0 0 -5]; % climb
%V_kb     = [0 0 2]; % decent
%V_kb     = [0 0 5]; % vortex ring state

% rho (density of air) [kg/m^3]
rho = 1.225  ;

%% Rotor geometry

% c (airfoil chord) [m]
c = 66.5 / 1000;

% R (rotor radius) [m]
R = 900 / 1000;

% Nb (number of blades)
Nb = 2;

%% Aerodynamic coefficients of NACA0015 [vanderWall2015, p. 128]

% lift slope
C_l_alpha = 5.75;

% Zero drag coefficent
C_d_0 = 0.0113;

% 1st order drag coefficent 
C_d_1 = 0.0;

% 2st order drag coefficent 
C_d_2 = 0.75;

%% Parameter 

% Correction Factor for figure of merit
kappa = 1;

% Effective dimensionless radius (often used with B = 0.97)
B = 1.0;

%% Basic calculation for known thrust and hub velocity ====================

% Calculation of rotor rotational speed [rad/s]
omega = 2 * pi * ( N / 60 );

% Calculation of blade tip velocity [m/s]
omega_R = omega * R;

% Calculation of rotor disk area
A = pi * R^2;

[v_i, v_h, v, V_c, P_i, P_c] = inducedVelocityWithUnits(T, V_kb, R, rho);

% Calculation of total flow
lambda = v / omega_R;

% Calculation of induced flow
lambda_i = v_i / omega_R;

% Calculation of induced flow in hover (often uses as reference value)
lambda_h = v_h / omega_R;

% Calculation of rotor advance ratio
mu = V_kb(1) / omega_R;

% Calculation of climb inflow ratio
mu_z = V_c / omega_R;

% Calculation of thrust coefficent
C_T         = T / (rho * A * omega_R ^ 2);

% Calculation of thrust coefficent
sigma       = Nb * c / ( pi * R );
FM          = sqrt (C_T^3  / 2) / ( (kappa * sqrt(C_T^3  / 2)) + (sigma * C_d_0 / 8) );

% Calculation of induced power coefficent
C_Pi        = C_T * lambda_i;

% Calculation of profile power coefficent
C_P0        = (sigma * C_d_0) / 8 * (1 + 1.5 * mu^2);

% Calculation of profile power
P_0         = C_P0 * rho * A * omega^3 * R^3;

% Calculation of torque
M_0         = P_0 /  omega;

format shortG

fprintf(' == Basic momentum theory ============================ \n')
fprintf('Thrust  = %3.3f N  \n', T)
fprintf('Power   = %3.3f W  \n', P_0 + P_i)
fprintf('Torque  = %3.3f Nm \n', (P_0 + P_i) / omega)
fprintf('\n')
fprintf('Profile power  = %3.3f W \n', P_0)
fprintf('Induced power  = %3.3f W \n', P_i)
fprintf('Climb power    = %3.3f W \n', P_c)
fprintf('Total power    = %3.3f W \n', P_0 + P_i + P_c)
fprintf('\n')
fprintf('Climb velocity            V_c = %3.3f m/s  \n', V_c)
fprintf('Induced velocity in hover v_h = %3.3f m/s  \n', v_h)
fprintf('Induced velocity          v_i = %3.3f m/s  \n', v_i)
fprintf('\n')
fprintf('Figure of merit = %3.3f %%  \n', FM*100)
fprintf('\n')

%% Blade element momentum theory - non stationary induced flow ============

% Notice that the induced flow lambda_i can be changed for the calculation
% of the collective pitch angle needed for the desired thrust coefficent
% C_T, as well as in the calculation of the thrust for given collective
% pitch and known inflow. Here the value from steady-state calculation from
% above is used for lambda_i. In a time-depend simulation you can use
% a custom model for dynamic induced inflow.

% Calculation of collective pitch angle for desired thrust and known inflow
[Theta_deg, ~, lambda_i] = bemThetaFromDesiredThrustCoefficentAndInflow(lambda_i, C_T, mu, mu_z, sigma, C_l_alpha);

% Calculation of collective pitch angle in radian
Theta_rad = Theta_deg/180*pi;

% Calculation of thrust for given collective pitch and known inflow
[T, M, R_kb, Q_kb, C_T, mu, mu_z] = bemThrustFromInflowParameters(...
    N, V_kb, lambda_i, Theta_rad, rho, R, c, Nb, C_l_alpha, ...
    C_d_0, C_d_1, C_d_2);

% Calculation of power coefficent according to 
% van der Wall, B.G.: Grundlagen der Hubschrauber-Aerodynamik.
% Springer-Verlag, 2015, p.181
C_P = C_T * lambda_h / B + sigma * C_d_0 / 8 ...
      + sigma * C_d_1 / 2 * (Theta_rad / 4 - lambda_h / ( 3 * B)) ...
      + sigma * C_d_2 / 2 * (Theta_rad^2 / 4 - 2 * Theta_rad * lambda_h / (3 * B) + lambda_h^2 / ( 2 * B^2 ));

  
fprintf(' == Blade element momentum theory - non stationary induced flow \n')
fprintf('Thrust  = %3.3f N  \n', C_T * (rho * A * omega_R ^ 2))
fprintf('Power   = %3.3f W  \n', C_P * (rho * A * omega_R ^ 3))
fprintf('Torque  = %3.3f Nm \n', C_P * (rho * A * omega_R ^ 3) / omega)
fprintf('\n')
fprintf('Collective Pitch Angle = %3.3f °  \n', Theta_rad / pi * 180)
fprintf('\n')

assert(Theta_rad / pi * 180 < 15, ...
    'Collective Pitch Angle too large! Theta > 15°') 

%% Blade element momentum theory - quasi-static ===========================

% Same calculation as above, but notice that the induced flow lambda_i
% is unknown and can not be changed.

fprintf('\n \n')
fprintf(' == Blade element momentum theory - quasi-static ============ \n')

% Calculation of Thrust and Torque with non linear solver.
% Numerical calculation of heave damping derivative and control sensitivty
% by finite differences
[Thrust, Torque, Zw, ZTheta] = bemResponseAndControlDerivativesNumPertub(N, Theta_rad, V_kb, C_l_alpha, C_d_0, C_d_1, C_d_2, B, R, c, Nb, rho);

% Calculation of thrust for given collective pitch and known inflow
[T, M, R_kb, Q_kb, C_T, mu, mu_z] = bemThrustFromInflowParameters(...
    N, V_kb, lambda_i, Theta_rad, rho, R, c, Nb, C_l_alpha, ...
    C_d_0, C_d_1, C_d_2);

fprintf('Thrust  = %3.3f N  \n', Thrust )
fprintf('Power   = %3.3f W  \n', Torque * omega)
fprintf('Torque  = %3.3f Nm \n', Torque)
fprintf('\n')
fprintf('Heave damping      (exact) Zw      = %3.3f Ns/m    \n', Zw)
fprintf('Control sensitivty (exact) Z_Theta = %3.3f Ns/deg  \n', ZTheta)

% Calculation of heave damping derivative and control sensitivty
% with analytical formula
[~, ~, ~, ~, Zw_analytic, ZTheta_analytic ] = bemResponseAndControlDerivatives(C_l_alpha, rho, omega_R, sigma, A, mu, lambda_i, mu_z);

fprintf('\n')
fprintf('Heave damping      (analytic) Zw      = %3.3f Ns/m    \n', Zw_analytic)
fprintf('Control sensitivty (analytic) Z_Theta = %3.3f Ns/deg  \n', ZTheta_analytic)


%% Heave damping derivative and control sensitivity
% From hover to fast forward flight

Velo_array = linspace(1,100);

Zw           = zeros(size(Velo_array));
ZTheta       = zeros(size(Velo_array));
Zw_h         = zeros(size(Velo_array));
Ztheta_h     = zeros(size(Velo_array));
Zw_ff        = zeros(size(Velo_array));
Ztheta_ff    = zeros(size(Velo_array));
Ztheta_analy = zeros(size(Velo_array));
Zw_analy     = zeros(size(Velo_array));

mu_vec       = zeros(size(Velo_array));

for ii = Velo_array
    
    V_kb_loop = V_kb;
    V_kb_loop(1) = ii-1;
    
    [T, ~, Zw(ii), ZTheta(ii)] = bemResponseAndControlDerivativesNumPertub(N, Theta_rad, V_kb_loop, C_l_alpha, C_d_0, C_d_1, C_d_2, B, R, c, Nb, rho);
    
    [v_i, v_h, v, V_c, P_i, P_c] = inducedVelocityWithUnits(T, V_kb_loop, R, rho);
      
    CT = abs(T) / (rho * A * omega_R ^ 2);
    
    lambda    = v / omega_R;
    lambda_i  = v_i / omega_R;
    lambda_h  = v_h / omega_R;
        
    V         = V_kb_loop(1);
    mu        = V / omega_R;
    mu_z      = V_c / omega_R;     
    
    % Calculate theoretical limit
    Zw_max    = -(C_l_alpha*sigma)/4  * (rho * A * omega_R ^ 2) / (omega_R);
    
    [Zw_h(ii), Ztheta_h(ii), Zw_ff(ii), Ztheta_ff(ii),  Zw_analy(ii), Ztheta_analy(ii), ] = bemResponseAndControlDerivatives(C_l_alpha, rho, omega_R, sigma, A, mu, lambda_i, mu_z);

    mu_vec(ii) = mu;
    
end

figure(1)
clf;
subplot(2,1,1)
hold on
plot(mu_vec, Zw)
plot(mu_vec, Zw_h)
plot(mu_vec, Zw_ff)
plot(mu_vec, Zw_analy, 'k--', 'LineWidth', 2)
plot([0 mu_vec(end)], [Zw_max Zw_max], 'r', 'LineWidth', 2)
hold off
title('Heave damping $Z_w$', 'Interpreter', 'latex')
xlabel('Rotor advance ratio $\mu$', 'Interpreter', 'latex')
ylabel('Heave damping ${N}/(m/s)$', 'Interpreter', 'latex')

legend({'exact solution','hover approx.', 'forward flight approx.', ...
       'analytical solution', 'theoretical limit'})

subplot(2,1,2)
hold on
plot(mu_vec, ZTheta)
plot(mu_vec, Ztheta_h)
plot(mu_vec, Ztheta_ff)
plot(mu_vec, Ztheta_analy, 'k--', 'LineWidth', 2)
hold off
title('Control sensitivity $Z_\Theta$', 'Interpreter', 'latex')
xlabel('Rotor advance ratio $\mu$', 'Interpreter', 'latex')
ylabel('Control sensitivity in $N/deg$', 'Interpreter', 'latex')

legend({'exact solution','hover approx.', 'forward flight approx.', 'analytical solution'})


