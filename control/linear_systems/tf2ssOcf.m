function [ A, B, C, D ] = tf2ssOcf( b, a ) %#codegen
% tf2ssOCF computes the state-space representation (ss) in observable
%   canonical form (OCF) from the coefficients of a transfer function (tf).
% 
% The following transfer function is considered:
% 
%         b_m*s^m + b_(m-1)*s^(m-1) + ... + b_1*s + b_0
% G(s) = ----------------------------------------------
%         a_n*s^n + a_(n-1)*s^(n-1) + ... + a_1*s + a_0
% 
% Inputs:
%   a       denominator coefficients (1 x n+1) vector (first element a_n)
%   b       numerator coefficients (1 x m+1) vector (first element b_m)
% 
% Outputs:
%   A       system (n x n) matrix
%   B       input (n x 1) matrix
%   C       output (1 x n) matrix
%   D       feedtrhough (1 x 1) matrix
% 

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

n = length(a)-1;
m = length(b)-1;
den = a(end:-1:2) / a(1);
num = b / a(1);
b_n = zeros(n,1);
if n>m
    b_n(end-(n-m)+1:-1:1) = num;
elseif n==m
    b_mod = num(1) * ( num/num(1) - a );
    b_n(end:-1:1) = b_mod(2:end);
else
    error('Check order of numerator and denominator.')
end

% observable canonical form
A = [ [ zeros(1,n-1); eye(n-1) ], -den' ];
B = b_n;
C = [ zeros(1,n-1), 1 ];
if n>m
    D = 0;
elseif n==m
    D = b(1);
end

end