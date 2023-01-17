function [ Thrust, Theta_1, V_A, alpha ] = aerodynamicsMulticopter( ...
    V_Kg,gamma, c_W_copter_seitlich, c_W_copter_oben, c_A_max, ...
    F_copter, m, g, u_Wg, rho, LF )
%aerodynamicsMulticopter   computes the necessary thrust for a multicopter 
%   On the basis of a simple aerodynamic model the thrust can be calculated
%   using the flight path velocity (V_Kg), the dimensionless coefficient
%   (C_W_copter_seitlich, c_W_copter_oben, c_A_max), the multicopter
%   specifications (A_copter, m) as well as the environmental conditions
%   (g, u_Wg, rho).
%
% Syntax:  [ Thrust, Theta_1, V_A, alpha ] = aerodynamicsMulticopter( ...
%    V_Kg,gamma, c_W_copter_seitlich, c_W_copter_oben, c_A_max, ...
%    A_copter, m, g, u_Wg, rho )
%
% Inputs:
%   u_Wg        lateral wind component (scalar), in m/s
%   V_Kg        flight path velocity (scalar), in m/s
%   gamma       angle of climb (scalar), in rad
%   c_W_copter_seitlich    lateral, dimensionless drag coefficient (scalar)
%               , -
%   c_W_copter_oben        upper, dimensionless drag coefficient (scalar), 
%               -
%   c_A_max     maximum lift coefficient of the multicopter (at an angle of
%               attack of +/- 45 degrees) (scalar), -
%   rho         air density (scalar), in kg/m^3
%   F_copter    upper front surface (scalar), in m^2
%   m           mass of the multicopter (scalar), in kg
%   g           gravity acceleration (scalar), in m/s^2
%   LF          load factor (scalar), -
%
% Outputs:
%   Thrust   	necessary thrust (scalar), in N
%   Theta_1  	banking angle (scalar), in rad
%   V_A         absolute velocity of the multicopter relative to the air
%               (scalar), in m/s
%   alpha    	angle between the rotor plane and the velocity vector 
%               relative to the air (scalar), in rad
%
% See also: propellerOp,  motorOp
%

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% If the lateral wind component is zero ...
if u_Wg == 0
    % set it to 0.01 m/s
    u_Wg = 0.01;
end

% Vertical component of the flight path velocity
w_Kg = -V_Kg * sin(gamma);
% lateral component of the flight path velocity
u_Kg = V_Kg * cos(gamma);

% The airspeed is the result of the flight path velocity relatively to the
% wind speed
V_A = sqrt((u_Kg + u_Wg)^2 + w_Kg^2); 
% This causes a new, aerodynamic angle of climb
gamma_a = atan(-w_Kg / (u_Kg + u_Wg));
% Initialising the banking angle and the counter
Theta_1 = 0;
Delta_Theta = 2;
i = 0;
% Conduct the calculation of the force components in x- and z-direction as
% long as the terminal condition is not accomplished
while Delta_Theta > 0.001*pi/180
    alpha = - gamma_a + Theta_1;
    % The total drag coefficient is the result of the lateral and upper
    % drag coefficient
    c_W = - (c_W_copter_oben - c_W_copter_seitlich)/2 * cos(2*alpha) + ...
        (c_W_copter_oben + c_W_copter_seitlich)/2;
    % The total lift coefficent
    c_A = c_A_max * sin(2*alpha);
    % By means of the coefficients, the dynamic pressure and the upper
    % front surface the aerodynamic forces of drag and lift can be computet
    W = c_W * rho/2 * V_A^2 * F_copter;
    A = c_A * rho/2 * V_A^2 * F_copter;
    % The sum of the all forces in x- and y- directions is the result of
    % the drag, lift and weight force
    X_g = - W * cos(gamma_a) - A * sin(gamma_a);
    Z_g = W * sin(gamma_a) - A * cos(gamma_a) + m*g*LF;
    % Recalculating the banking angle with the force components
    Theta_2 = -atan(-X_g / Z_g);
    Delta_Theta = abs(Theta_2 - Theta_1);
    Theta_1 = Theta_2;
    i = i + 1;
end
% Necessary thrust
Thrust = sqrt(X_g^2 + Z_g^2);

end