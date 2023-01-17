
% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Fabian Guecker
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

%%
clc;
clear;

syms ax bx cx dx t 'real'
syms ay by cy dy t 'real'
syms az bz cz dz t 'real'

cubic_spline_x = ax + bx*t + cx*t^2 + dx*t^3
cubic_spline_y = ay + by*t + cy*t^2 + dy*t^3
cubic_spline_z = az + bz*t + cz*t^2 + dz*t^3

d_cubic_spline_x = diff(cubic_spline_x, 't')
d_cubic_spline_y = diff(cubic_spline_y, 't')
d_cubic_spline_z = diff(cubic_spline_z, 't')

arc_length = int( sqrt( d_cubic_spline_x^2 + d_cubic_spline_y^2 + d_cubic_spline_z^2), t)

arc_length_approx = taylor(arc_length,t,'ExpansionPoint',0,'Order',4)
arc_length_approx_d = diff(arc_length_approx, 't')
