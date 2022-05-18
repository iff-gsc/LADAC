function [y] = polyVal(p, x)
%POLYVAL Evaluate polynomial.
%   Y = POLYVAL(P,X) returns the value of a polynomial P evaluated at X. P
%   is a vector of length N+1 whose elements are the coefficients of the
%   polynomial in descending powers.
%
%       Y = P(1)*X^N + P(2)*X^(N-1) + ... + P(N)*X + P(N+1)
%
% Inputs:
%
%   p               coefficients of the polynomial in descending powers
%                   (vector 1xN)
%
%   x               points X, where the polynomial P is evaluated  
%                   (vector 1xM)
%
% Outputs:
%
%   y               values of the evaluated points X of the polynomial P 
%                   (vector 1xM)
%
% Syntax:
%   [ y ] = polyVal(p, x)
%

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Fabian Guecker
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

nc = length(p);
siz_x = size(x);
y = zeros(siz_x);

if nc > 0
    y(:) = p(1);
end

for i = 2:nc
    y(:) = x .* y + p(i);
end

end