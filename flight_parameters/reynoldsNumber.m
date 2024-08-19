function Re = reynoldsNumber( rho, V, l, mu ) %#codegen
% reynoldsNumber computes the Reynolds number
% 
% Inputs:
%   rho         air density, in kg/m^3
%   V           air velocity, in m/s
%   l           characteristic length of the body, in m
%   mu          air dynamic viscosity, in kg/(m*s)
% 
% Outputs:
%   Re          Reynolds number, in 1
% 

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

Re = rho .* V .* l ./ mu;

end