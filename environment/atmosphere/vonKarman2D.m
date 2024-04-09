function [ wg, wg_big, x ] = vonKarman2D( x_max, y_max, Delta_x, ...
    Delta_y, Y, sigma, L, seed, is_symmetric )
% vonKarman2D two-dimensional turbulence field with Fourier transform
%   Only the vertical wind is modeled.
%   Since von Karman noise cannot be represented exactly by filtered white
%   noise the turbulence is generated a-priory by fast Fourier transform.
%   In simulation the current wind must be interpolated from the turbulence
%   field. This makes the model relatively computationally intensive.
% 
% Syntax:
%   [ wg, wg_big, x ] = vonKarman2D( x_max, y_max, Delta_x, Delta_y, Y,
%                           sigma, L, seed, is_symmetric )
% 
% Inputs:
%   x_max           Longitudinal length of the turbulence field (scalar), m
%   y_max           Lateral width (m) - should be greater than L
%   Delta_x         Longitudinal position resolution (scalar), m
%   Delta_y         Lateral position resolution (scalar), m
%   Y               Lateral position of interest (1xP array), m
%   sigma           Turbulence velocity variance (scalar), m/s
%   L               Scale length at medium/high altitudes (scalar), m
%                   (typically 762m)
%   seed            Noise seed (scalar)
%   is_symmetric    Defines whether the wind field is symmetric in lateral
%                   direction (boolean)
% 
% Outputs:
%   wg              Vertical velocity of 2D wind field at lateral positions
%                   of interest (MxP array, where M=x_max/Delta_x and
%                   P=length(Y)), m/s
%   wg_big          Vertical velocity of 2D wind field
%                   (MxN array, where M=x_max/Delta_x and N=y_max/Delta_x),
%                   m/s
%   x               Forward position sampling (1xN array, where
%                   M=x_max/Delta_x), m
% 
% See also:
%   vonKarman1D, turbulenceIntensityRMS
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

a = 1.339;

if Delta_x <=0 || Delta_y <= 0
    error('Position resolution must be positive.');
end

% Spatial sampling period
x = 0:Delta_x:x_max;
y_max = 0.5*y_max;
Delta_y_max = mod(y_max,Delta_y);
y_max = y_max-Delta_y_max;
y = -y_max:Delta_y:y_max;
if y_max<Delta_y
    error('Lateral length too small.');
end

rng(seed);
wn = 2*pi * randn( length(x), length(y) ) / sqrt(Delta_x) / sqrt(Delta_y);

if is_symmetric
    wn(:,y>0) = flip(wn(:,y<0),2);
end

% Sampling frequency
fs_x = 1/Delta_x;
fs_y = 1/Delta_y;

% Signal length
L_x = length(x);
L_y = length(y);

% Frequency
f_x = fs_x/L_x*(-L_x/2:(L_x/2)-1)';
f_y = fs_y/L_y*(-L_y/2:(L_y/2)-1)';

% Angular frequency
Omega_x = 2*pi*f_x;
Omega_y = 2*pi*f_y;

[ omegax, omegay ] = meshgrid( Omega_y, Omega_x );

% [1], p. 539
S = 4*sigma^2*(a*L)^4/(9*pi)*(omegax.^2+omegay.^2) ...
    ./ ((1+(a*L)^2*(omegax.^2+omegay.^2)).^(7/3));

% [2], Eq. (11)
wg_big = ifft2( fftshift(sqrt(S)).*fft2(wn), 'symmetric' );

wg = interp1(y',wg_big',Y)';

end