function [c_L,c_m,alpha_E,x_dt] = unstAirfoilWagner( ...
    V, c, C_L_alpha, x_ac, x, alpha ) %#codegen
% unstAirfoilWagner computes unsteady airfoil aerodynamics according to the
% Wagner function.
% 
% Inputs:
%   V           Airspeed (1xn vector), in m/s
%   c           Chord (1xn vector), in m
%   C_L_alpha   Lift curve slope (1xn vector), in 1/rad
%   x_ac        Aerodynamic center measured from the nose of the profile
%               (1xn vector), in m
%   x           State (2xn matrix)
%   alpha       Angle of attack (1xn vector), in rad
% 
% Outputs:
%   c_L         Circulatory lift coefficient (1xn vector)
%   c_m         Circulatory pitching moment coefficient (1xn vector)
%   alpha_E     Effective angle of attack (1xn vector) corresponding to 
%               c_L_c, rad
%   x_dt        Time-derivative of the state x (2xn matrix)
% 
% Literature:
%   [1] Leishman, J. G., and Nguyen, K. Q. (1990). State-space 
%       representation of unsteady airfoil behavior. AIAA journal, 28(5),
%       836-844. https://arc.aiaa.org/doi/pdf/10.2514/3.25127
%   [2] Hansen, M. H., Gaunaa, M., & Madsen, H. A. (2004). A Beddoes-
%       Leishman type dynamic stall model in state-space and indicial
%       formulations.
%       https://backend.orbit.dtu.dk/ws/portalfiles/portal/7711084/ris_r_1354.pdf
% 
% Authors:
%   Yannic Beyer
% 
% See also:
%   unstAirfoilAero

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2024 Yannic Beyer
%   Copyright (C) 2024 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************


% https://arc.aiaa.org/doi/pdf/10.2514/3.25127
A_1 = 0.165;
A_2 = 0.335;
b_1 = 0.0455;
b_2 = 0.3;

T_1 = c./(2*V*b_1);
T_2 = c./(2*V*b_2);

x_dt = zeros(2,size(x,2));
x_dt(1,:) = 1./T_1 .* (alpha - x(1,:));
x_dt(2,:) = 1./T_2 .* (alpha - x(2,:));

alpha_E = 0.5*alpha + A_1.*x(1,:) + A_2.*x(2,:);
c_L = C_L_alpha .* alpha_E;
c_m = c_L .* (0.25-x_ac); % + c_m0;

end