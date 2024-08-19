% Script for generation of analytic expressions used in trajGetMatch and
% and trajGetMatchEnhanced functions.

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Fabian Guecker
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% Clear workspace and console
clc;
clear;

% Degree of polynominal
degree = 3;

% Divide every expression by this factor to reduce calculation overhead
divide = 2;

% Symbolic variable defintions
syms xf yf zf real
syms t real
x = sym('x', [1 degree+1], 'real');
y = sym('y', [1 degree+1], 'real');
z = sym('z', [1 degree+1], 'real');

% Position of the vehicle
Pf = [xf; yf; zf];

% Polynomial of the trajectory first element contains the highest degree
% coefficient
px = poly2sym(x, t);
py = poly2sym(y, t);
pz = poly2sym(z, t);

% Polynominal vector
Pt = [px; py; pz];

% distance vector
dist = (Pt - Pf);

% squared euclidean distance
dist_norm_square = dist(1)^2 + dist(2)^2 + dist(3)^2;

% Expand expression
dist_norm_square_dt = expand(diff(dist_norm_square, t));

% Simplify expression
dist_norm_square_dt = simplify(dist_norm_square_dt, 'steps', 10);

% Factor expression and return polynomial coefficients for trajGetMatch
% function. The coefficient vector contains the polynomial coefficients
% and the possible solutions can be found with roots(coefficients)
% x1,x2,x3,..,.. correspond to the spline coefficients and have to be
% replaced with x(1), x(2), x(3). The first element contains always the
% highest coefficient belonging to the highest degree in the polynomial.

coefficent = (fliplr(coeffs(dist_norm_square_dt, t))/divide)'


% trajGetMatchEnhanced for validation purposes only

%derivative_1st = diff(dist_norm_square, t)
% derivative_2st = diff(derivative_1st, t)


