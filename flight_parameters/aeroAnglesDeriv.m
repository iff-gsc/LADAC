% aeroAnglesDeriv computes the time derivative of the aerodynamic
% angles.
%   The time derivative of the aerodynamic angles (angle of attack and
%   sideslip angle) can be computed by deriving their functional
%   definition according to the ISO 1151 (or LN9300).
% 
% Literature:
% [1]   Stevens, B. L. et al. (2016): Aircraft Control and Simulation.
%       Dynamics, Control Design, and Autonomous Systems. 3rd ed. Wiley.
% 
% Inputs:
%   V_Ab            three dimensional airspeed vector in body-fixed frame
%                   (b), m/s
%   dot_V_Ab        three dimensional time derivative of the airspeed
%                   vector in body-fixed frame (b), m/s^2
% 
% Outputs:
%   dot_alpha       scalar time derivative of the angle of attack, rad
%   dot_beta        scalar time derivative of the sideslip angle, rad
% 
% See also: aeroAngles
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

function [ dot_alpha, dot_beta ] = aeroAnglesDeriv( ...
    V_Ab, dot_V_Ab ) %#codegen

% compute squared values (multiple use)
u_Ab_2 = V_Ab(1,:).^2;
w_Ab_2 = V_Ab(3,:).^2;

% compute the time derivative of the angle of attack according to [1, page 
% 81]
dot_alpha = ( V_Ab(1,:).*dot_V_Ab(3,:) - V_Ab(3,:).*dot_V_Ab(1,:) ) ./ ...
    ( u_Ab_2 + w_Ab_2 );

% compute the time derivative of the airspeed according to [1, page 81]
V_A = sqrtReal( u_Ab_2 + V_Ab(2,:).^2 + w_Ab_2 );
dot_V_A = ( V_Ab(1,:).*dot_V_Ab(1,:) + V_Ab(2,:).*dot_V_Ab(2,:) + ...
    V_Ab(3,:).*dot_V_Ab(3,:) ) ./ V_A;

% compute the time derivative of the sideslip angle according to [1, page 
% 81]
dot_beta = ( dot_V_Ab(1,:).*V_A - V_Ab(1,:).*dot_V_A ) ./ ...
    ( V_A .* sqrtReal( V_Ab(1,:).^2 + V_Ab(3,:).^2 ) );

end