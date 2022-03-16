function [Zw_h, Ztheta_h, Zw_ff, Ztheta_ff, Zw_analytical, ...
    Ztheta_analytical] = bemResponseAndControlDerivatives(C_l_alpha, ...
    rho, omega_R, sigma, A, mu, lambda_i, mu_z)

% bemResponseAndControlDerivatives computes the heave damping derivative 
% and control sensitivity with dimensions for given rotor geometry and 
% known inflow conditions.
%
%   The computation is based on momentum theory and steady-state.
%   The calculation includes advanced approximation for hover and forward
%   flight that take also the climb rate into account.
%   An analytic expression is used for high fidelity values from hover to
%   fast forward flight [1].
%   The calculation does not cover the vortex ring state, because all
%   expressions are based on momentum theory without VRS-model.
%
% Literature:
% [1]   Gücker, F. (2020): Lage- und Bahnregelung eines drehzahlsynchronen
%          vier-rotorigen Drehflüglers mit kollektiver Blattverstellung.
%          Master-Thesis,  TU Braunschweig
%
% Syntax: [Zw_h, Ztheta_h, Zw_ff, Ztheta_ff, Zw_analytical,
%   Ztheta_analytical] = bemResponseAndControlDerivatives(R_b, ...
%   Q_b, lambda_vec, rotor_position, R)
%
% Inputs:
%   C_l_alpha         
%
%   rho               air density. (scalar), in kg/m^3
%   omega_R           rotor blade tip speed. (scalar), in m/s
%   sigma             rotor solidity. (scalar), dimensionless
%   A                 rotor disk area (scalar), in m^2
%   mu                rotor advance ratio. (scalar), dimensionless
%   lambda_i          induced flow ratio. (scalar), dimensionless
%   mu_z              climb inflow ratio. (scalar), dimensionless
%
% Outputs:
%   Zw_h              Heave damping derivative approximation for hover.
%                     (scalar), in N / (m/s)
%   Ztheta_h          Control sensitivity approximation for hover.
%                     (scalar), in N / deg
%
%   Zw_ff             Heave damping derivative approximation for 
%                     forward flight. (scalar), in N / (m/s)
%   Ztheta_ff         Control sensitivity approximation for forward flight.
%                     (scalar), in N / deg
%
%   Zw_analytical     Heave damping derivative approximation for hover and
%                     forward flight. (scalar), in N / (m/s)
%   Ztheta_analytical Control sensitivity approximation for hover and
%                     forward flight. (scalar), in N / deg
%

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Fabian Guecker
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

uz   = mu_z;

%% Hover ==================================================================

% Heave damping derivative approximation according [1, p. XXX]
d_CT_d_mu =    -(2*C_l_alpha*sigma*(lambda_i + uz)) / ...
                (16*lambda_i + 8*uz + C_l_alpha*sigma);

% Convert units to  [N / (m/s)]            
Zw_h      =    d_CT_d_mu * (rho * A * omega_R ^ 2) / (omega_R);


% Control sensitivity approximation according [1, p. XXX]
d_CT_d_Theta =  (4*C_l_alpha*sigma*(2*lambda_i + uz)) /...
                (3*(16*lambda_i + 8*uz + C_l_alpha*sigma));
                            
% Convert units to [N / deg]            
Ztheta_h =  -d_CT_d_Theta * (rho * A * omega_R ^ 2) * pi / 180;
 
%% Forward Flight =========================================================

% Heave damping derivative approximation according [1, p. XXX]
d_CT_d_mu =    (2*C_l_alpha*mu*sigma) / ...
               (8*mu + C_l_alpha*sigma);

% Convert units to  [N / (m/s)]            
Zw_ff      =    -d_CT_d_mu * (rho * A * omega_R ^ 2) / (omega_R);


% Control sensitivity approximation according [1, p. XXX]
d_CT_d_Theta = (2*C_l_alpha*mu*sigma*(2 + 3*mu^2))/(24*mu + 3*C_l_alpha*sigma);

% Convert units to  [N / deg]            
Ztheta_ff =  -d_CT_d_Theta * (rho * A * omega_R ^ 2) * pi / 180;


%% Heave damping derivative for hover and forward flight ==================

% Heave damping derivative approximation according [1, p. XXX]
d_CT_d_mu_analytical = C_l_alpha*sigma*sqrt(  (2*lambda_i + 2*uz)^2 + (2*mu)^2) / ( sqrt( (16*lambda_i + 8*uz)^2  + (8*mu)^2 ) + C_l_alpha*sigma);

% Convert units to [N / deg]      
Zw_analytical = -d_CT_d_mu_analytical * (rho * A * omega_R ^ 2) / (omega_R);


% Control sensitivity approximation according [1, p. XXX]
d_CT_d_Theta_analytical = C_l_alpha*sigma*sqrt(  (4*mu + 6*mu^3)^2 + (8*lambda_i + 4*uz)^2) / ( sqrt((24*mu)^2  + (48*lambda_i + 24*uz)^2) + 3*C_l_alpha*sigma);

% Convert units to [N / deg]      
Ztheta_analytical = -d_CT_d_Theta_analytical * (rho * A * omega_R ^ 2) * pi / 180;

end

