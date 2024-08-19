function [ F_sub, M_sub ] = NACA0015_airfoil(V_sub, span, coord, rho)

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Fabian Guecker
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

V_x = V_sub(1);
V_z = V_sub(3);

% Flaeche in [m^2]
area = span * coord;

% aerodynamischer Anstellwinkel
alpha_a = -atan2(V_z, V_x);

x = alpha_a * 180 / pi;

% Auftriebsbeiwert
c_l = (0.2118*x)/((0.09377*x)^8+1.963)+1.05*sin(0.0349065*x)*(1-1/((0.05374*x)^8+0.9658));

% Widerstandsbeiwert
c_d = (-0.85*cos(pi/90*x)+0.9)*(1-1/((0.0767*x)^12+1.3));

%Geschwindigkeit
V = sqrt(V_x^2 + V_z^2);

% Einstroemwinkel (Einbauwinkel) phi
phi = -alpha_a;
   
Fz = 0.5 * rho  *V^2 * area * ( c_l * cos(phi) - c_d * sin(phi) );
  
Fx = 0.5 * rho * V^2 * area * ( c_d * cos(phi) + c_l * sin(phi) );
        
F_sub = [-Fx, 0, Fz]';

M_sub = [0, 0, 0]';

end

