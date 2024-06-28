function [ Wg, x ] = vonKarman1D( x_max, Delta_x, sigma, L, seeds )
% vonKarman1D one-dimensional turbulence field with Fourier transform
% 
% Syntax:
%   [ Wg, x ] = vonKarman1D( x_max, Delta_x, sigma, L, seed )
% 
% Inputs:
%   x_max           Length of the turbulence field (scalar), m
%   Delta_x         Position resolution (scalar), m
%   sigma           Turbulence velocity variance (scalar), m/s
%   L               Scale length at medium/high altitudes (scalar), m
%                   (typically 762m)
%   seeds           Noise seeds in all directions (1x3 array)
% 
% Outputs:
%   Wg              1D wind field in forward-right-down directions
%                   (3xN array, where N=x_max/Delta_x), m/s
%   x               Forward position sampling (1xN array, where
%                   N=x_max/Delta_x), m
% 
% See also:
%   vonKarman2D, turbulenceIntensityRMS
% 
% Literature:
%   [1] Etkin, B. (2005). Dynamics of Atmospheric Flight. Dover
%       Publications Inc. pp. 539.
%   [2] Wang, X., Van Kampen, E., Chu, Q. P., & De Breuker, R. (2019).
%       Flexible aircraft gust load alleviation with incremental nonlinear
%       dynamic inversion. Journal of Guidance, Control, and Dynamics,
%       42(7), 1519-1536.

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2023 Yannic Beyer
%   Copyright (C) 2023 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

x = 0:Delta_x:x_max;

if length(seeds) ~= 3
    error('Three seeds must be specified.')
end

rng(seeds(1));
wnu = sqrt(2*pi) * randn( 1, length(x) ) / sqrt(Delta_x);
rng(seeds(2));
wnv = sqrt(2*pi) * randn( 1, length(x) ) / sqrt(Delta_x);
rng(seeds(3));
wnw = sqrt(2*pi) * randn( 1, length(x) ) / sqrt(Delta_x);

a = 1.339;

% Sampling frequency
fs_x = 1/Delta_x;

% Signal length
L_x = length(x);

% Frequency
f_x = fs_x/L_x*(-L_x/2:1:(L_x/2)-1);
% f_x = fs_x/(2*L_x-1)*(0:L_x-1);


% Angular frequency
Omega_x = 2*pi*f_x;

% [1], p. 539
Su = sigma^2*L/pi ./ ((1+(a*L*Omega_x).^2).^(5/6));
Sw = sigma^2*L/(2*pi)*(1+8/3*(a*L*Omega_x).^2)./((1+(a*L*Omega_x).^2).^(11/6));

Wg = [ ...
    ifft( sqrt(fftshift(Su)).*fft(wnu), 'symmetric' ); ...
    ifft( sqrt(fftshift(Su)).*fft(wnv), 'symmetric' ); ...
    ifft( sqrt(fftshift(Sw)).*fft(wnw), 'symmetric' ); ...
    ];

end