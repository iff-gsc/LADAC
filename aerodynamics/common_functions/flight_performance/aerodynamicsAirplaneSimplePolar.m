function [ Thrust, E ] = ...
    aerodynamicsAirplaneSimplePolar( m, g, A_star, E_star, V_star, ...
    rho_star, C_A_0, V, gamma, rho, LF )
%aerodynamicsAirplaneSimplePolar computes the necessary thrust for an 
%   airplane based on a quadratic polar for the drag coefficient depending
%   on the lift coefficient. The quadratic polar is defined by a design
%   lift, a design maximum glide ratio and a design dynamic pressure
%   (computed from design airspeed and design air density). From the
%   quadratic polar the drag is computed. The required thrust is computed
%   from force equilibirum considering a flight path angle and assuming the
%   force vector to be parallel to the flight path.
%
% Literature: 
%   [1] Gerhard Bruening, Xaver Hafer und Gottfried Sachs. Flugleistungen: 
%       Grundlagen, Flugzustaende, Flugabschnitte Aufgaben und Loesungen. 
%       Zweite, neubearbeitete und erweiterte Auflage. Hochschultext.Berlin
%       und Heidelberg: Springer, 1986. isbn: 978-3-540-16982-6. 
%
% Syntax:  [ Thrust, E ] = ...
%   aerodynamicsAirplaneSimplePolar( m, g, A_star, E_star, V_star, ...
% 	rho_star, C_A_0, V, gamma, rho, LF )
%
% Inputs:
%   m           mass of the airplane (scalar), in kg
%   g           gravity acceleration (scalar), in m/s^2
%   A_star      design lift (scalar), in N
%   E_star      glide ratio in design state (scalar), -
%   V_star      velocity in design state (scalar), in m/s
%   rho_star	air density in design state (scalar), in kg/m^3
%   C_A_0       minimum drag lift coefficient, in -
%   gamma       angle of climb (scalar), in rad
%   rho         air density (scalar), in kg/m^3   
%   LF          load factor (scalar), -
%
% Outputs:
%   Thrust		necessary thrust (scalar), in N
%   V_A         absolute velocity of the airplane relative to the air
%               (scalar), in m/s
%   SoF_aiplane state of flight of the airplane (scalar), -
%
% See also: aerodynamicsAirplaneConstantE
%

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************
			
% compute the drag for maximum glide ratio and design lift
W_star = A_star / E_star;	

% set reference surface to 1 (not relevant for the result)
S = 1;

% compute design dynamic pressure
q_star = rho_star/2 * V_star^2;

% compute lift coefficient for maximum glide ratio
C_A_star = A_star / ( q_star * S );

% compute drag coefficient for maximum glide ratio
C_W_star = W_star / ( q_star * S );

% compute zero lift drag coefficient according to a quadratic polar
% according to [1, page 82] (attention: in the book epsilon = 1/E)
C_W_0 = 0.5 * C_W_star / ( 1 - C_A_0 );

% The lift can be calculated from force equilibirum
A = cos(gamma)* m*g * LF;	

% compute actual dynamic pressure
q = rho/2 * V^2;

% compute actual lift coefficient and avoid devision by zero
if q == 0
    C_A = 0;
else
    C_A = A / ( q * S );
end

% compute faktor for the slope of the quadratic polar
k = C_W_0 / C_A_star^2;

% compute actual drag coefficient
C_W = C_W_0 + k * C_A^2;

% compute actual glide ratio
E = C_A / C_W;

% compute actual drag
W = C_W * q * S;

% compute required thrust from force equilibrium
Thrust = W + m*g * sin(gamma);

end

