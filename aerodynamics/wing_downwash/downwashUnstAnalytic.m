function w_dw = downwashUnstAnalytic( V, l, l_1, z, t, t_0 )
% downwashUnstAnalytic compute the downwash of an airfoil on a following
% airfoil.
%   The downwash is computed according to [1], eq. (4.2.14)
% 
% Inputs:
%   V           airspeed (scalar), in m/s
%   l           horizontal distance from front airfoil trailing edge to aft
%               airfoil leading edge (scalar) (see [1], Fig. 4.5), in m
%   l_1         horizontal distance from aerodynamic center of the front
%               airfoil to the trailing edge of the aft airfoil (scalar)
%               (see [1], Fig. 4.5), in m
%   z           vertical distance from front airfoil to aft airfoil (see
%               [1], Fig. 4.5), in m
%   t           time (array), in s
%   t_0         time shift (scalar), in s
% 
% Outputs:
%   w_dw        induced downwash velocity of the front airfoil on the aft
%               airfoil, in m/s
% 
% Literature:
%   [1] Andrews, S. P. (2011). Modelling and simulation of flexible 
%       aircraft: handling qualities with active load control (Doctoral
%       dissertation, Cranfield University).
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

gamma = 1;
w_dw = zeros(size(t));
% t_0 = l_1/V;
idx = t>=t_0;
w_dw(~idx) = 0;
w_dw(idx) = gamma/(2*pi) * ( (l-V*(t(idx)-t_0))./(z^2+(l-V*(t(idx)-t_0)).^2) - l_1/(z^2+l_1^2) );

end