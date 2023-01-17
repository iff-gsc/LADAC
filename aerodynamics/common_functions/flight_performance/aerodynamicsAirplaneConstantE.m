function [ Thrust, V_A, SoF_airplane ] = ...
    aerodynamicsAirplaneConstantE( m, g, E_star, V_star, rho_star, gamma, rho, LF )
%aerodynamicsAirplane computes the necessary thrust for an airplane
%     On the basis of a simple aerodynamic model the thrust for an 
%     airplane climb is being calculate using the design state in a hori-
%     zontal flight (E_star, V_star and rho_star) [1]. The influence of 
%     lateral crosswinds is not taking into account, since they do not have
%     an effect on grdeability but rather on the distance travelled above 
%     ground an the angle of climb [2, S.241- S.242].
%     alpha = sigma = beta = 0 can be applied. 
%
% Literature: 
%   [1] Gerhard Bruening, Xaver Hafer und Gottfried Sachs. Flugleistungen: 
%       Grundlagen, Flugzustaende, Flugabschnitte Aufgaben und Loesungen. 
%       Zweite, neubearbeitete und erweiterte Auflage. Hochschultext.Berlin
%       und Heidelberg: Springer, 1986. isbn: 978-3-540-16982-6. 
%   [2] Joachim Scheiderer. Angewandte Flugleistung: Eine Einfuehrung in 
%       die operationelle Flugleistung vom Start bis zur Landung. 1. Aufl. 
%       Berlin: Springer, 2008. isbn: 978-3-540-72724-8.
%
% Syntax:  [ Thrust, V_A, Flugzustand_Flaechenflzg ] = ...
%    aerodynamicsAirplane( m, g, E_star, V_star, rho_star, gamma, rho )
%
% Inputs:
%    m          mass of the airplane (scalar), in kg
%    g          gravity acceleration (scalar), in m/s^2
%    E_star     glide ratio in design state (scalar), -
%    V_star     velocity in design state (scalar), in m/s
%    rho_star	air density in design state (scalar), in kg/m^3
%    gamma      angle of climb (scalar), in rad
%    rho		air density (scalar), in kg/m^3   
%    LF         load factor (scalar), -
%
% Outputs:
%    Thrust		necessary thrust (scalar), in N
%    V_A		absolute velocity of the airplane relative to the air
%               (scalar), in m/s
%    SoF_aiplane state of flight of the airplane (scalar), -
%
% See also: propellerOp,  motorOp
%

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% Calculating the lift in design state, horizontal flight (gamma = 0)
A_star = m*g*LF;					

% With E_star the drag can be calculated
W_star = A_star / E_star;	

% The drag for a flight with the optimal glide ratio is exactly half of the
% total drag
W_0_star = 0.5 * W_star;			

% By means of the velocity in design state the airspeed can be calculated
% using the equation for the lift coefficient and under the assumption of a
% constant lift coefficient
V = V_star * sqrt(cos(gamma) * rho_star/rho);		

% Scaling the drag for a flight with the optimal glide ratio with the
% dynamic pressure
W_0 = W_0_star * (V^2*rho/2)/(V_star^2*rho_star/2);	 

% The lift can be calculated using the lift equation
A = cos(gamma)* m*g;		

% With the aid of the glide ratio the drag can be calculated
W = A / E_star;					

% Initializing the control variable to assess the state of flight
% = 0 --> efficient climb
SoF_airplane = 0;		


if W_0 < W && W < 2*W_0				
	
    % State of flight in between optimal flight and climb --> grey area, 
    % leaving area of optimal climb

    SoF_airplane = 1;		% --> flight in grey zone according to theory
    
elseif  W < W_0				

    % Transition into vertical climb

    SoF_airplane = 2;		% --> vertical climb
    
    W = W_0;
    E_star = A / W_0;

end 

% Necessary Thrust
Thrust = m*g * ( sin(gamma) + 1/E_star *cos(gamma));	

% Actual airspeed matches current airspeed
V_A = V;		



end

