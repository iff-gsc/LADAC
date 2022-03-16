function [c_L_c,z_dt] = airfoilFlapUnstLift(V,Ma,c,delta_qs,z) %#codegen
% airfoilFlapUnstLift implements an unsteady lift coefficient model for a
% flap according to Leishman [1].
%   The function returns the circulatory part of the lift coefficient as
%   well as the time derivative of the required state.
% 
% Inputs:
%   V           airspeed (1xN array), in m/s
%   Ma          Mach number (1xN array), dimensionless
%   c           airfoil chord (1xN array), in m
%   delta_qs    quasisteady angle of attack due to the imposed flap
%               deflection [1], eq. (5), see airfoilFlapDeltaQs
%               (1xN array), in rad
%   z           state variable (2xN array)
% 
% Outputs:
%   c_L_c       circulatory part of the unsteady lift coefficient due to
%               the flap (1xN array), dimensionless
%   z_dt        time derivative of the state variable (2xN array)
% 
% Literature:
%   [1] Leishman, J. G. (1994). Unsteady lift of a flapped airfoil by
%       indicial concepts. Journal of Aircraft, 31(2), 288-297.
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

A_1 = 0.3;
A_2 = 0.7;
b_1 = 0.14;
b_2 = 0.53;

beta = sqrtReal(1-Ma.^2);

% [1], eq. (39)
z_dt = zeros(2,length(V));
z_dt(1,:) = z(2,:);
z_dt(2,:) = -b_1*b_2*(2*V./c).^2.*beta.^4 .* z(1,:) + ...
    -(b_1+b_2)*(2*V./c).*beta.^2 .* z(2,:) + delta_qs;

% [1], eq. (40)
c_L_c = 2*pi./beta .* ( ...
    (b_1*b_2)*(2*V./c).^2.*beta.^4 .* z(1,:) + ...
    (A_1*b_1+A_2*b_2)*(2*V./c).*beta.^2 .* z(2,:) ...
    );

end

