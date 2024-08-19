function [ Q ] = integralSimpson( func, A, B, steps)

% integralSimpson Numerically evaluate integral with Simpson quadrature.
%   Q = integralSimpson(FUN, A, B, steps) attempts to approximate the 
%   integral of scalar-valued function FUN from A to B. The interval given 
%   by A and B is equally spaced with a constant stepsize.
%
% Inputs:
%
%   func            function handle Y = FUN(X) should accept a scalar
%                   argument X and return a scalar argument Y.
%                   (scalar function handle)
%
%   A               lower bound for integration
%                   (scalar)
%
%   B               Upper bound for integration
%                   (scalar)
%
%   steps           Number of subsegments to evaluate
%                   (integer)

%
% Outputs:
%
%   Q               integration result
%
% Syntax:
%   [ Q ] = integralSimpson( func, A, B, steps)
%

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Fabian Guecker
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

t = A; 
Q = 0;
step = (B-A) / steps;
step_1_2 = 0.5 * step;

Q = 0.5 * func(A);
Q = Q - 0.5 * func(B);

for i = 1:steps
    t = t + step;
    Q = Q + 2.0 * func(t - step_1_2) + func(t);   
end

Q = (1/3) * step * Q;

end