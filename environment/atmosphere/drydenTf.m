function [b,a] = drydenTf( sigma, L, V, axis ) %#codegen
% drydenTf computes the coefficients of the transfer function of the
%   Dryden wind turbulence spectrum.
% 
% Inputs:
%   sigma       root-mean-square wind turbulence intensity, in m/s
%   L           wind scale length, in m
%   V           airspeed, in m/s
%   axis        a string specifying the direction ('u', 'v' or 'w')
% 
% Outputs:
%   a           denominator coefficients (1 x n+1) vector (first element a_n)
%   b           numerator coefficients (1 x m+1) vector (first element b_m)      
% 
% See also:
%   turbulenceIntensityRMS, turbulenceScaleLength, tf2ssOcf
% 

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

switch axis
    case 'u'
        b = [ sigma * sqrt( 2*L/pi/V ) ];
        a = [ L/V, 1 ];
    case 'v'
        b = sigma * sqrt( L/pi/V ) * [ sqrt(3)*L/V, 1 ];
        a = [ L^2/V^2, 2*L/V, 1 ];
    case 'w'
        b = sigma * sqrt( L/pi/V ) * [ sqrt(3)*L/V, 1 ];
        a = [ L^2/V^2, 2*L/V, 1 ];
end

end