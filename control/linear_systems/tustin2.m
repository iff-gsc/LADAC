function [Ad,Bd] = tustin2(A,B,T)
% tustin2 is a fast implementation for transforming a continuous second-
% order state-space model to a discrete one using the efficient Tustin
% transformation.
% 
% Inputs:
%   A           2x2 system matrix with the format: [ 0, 1; a21, a22 ]
%   B           2x1 input matrix with the format: [ 0; b2 ]
%   T           Sample time, in s
% 
% Outputs:
%   Ad          2x2 discrete system matrix
%   Bd          2x1 discrete input matrix

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2025 Yannic Beyer
%   Copyright (C) 2025 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

if ~( A(1,1)==0 && A(1,2)==1 && isequal(size(A),[2,2]) && B(1)==0 && ...
        isequal(size(B),[2,1]) )
    error('The given matrix does not match the required format.');
end

h = T/2;
h2 = h^2;
a22 = A(2,2);
a21 = A(2,1);
D = 1-a22*h-a21*h2;
if abs(D) < eps(D)
    D = eps(D);
end
Dinv = 1/D;
Ad = Dinv * [1-a22*h+a21*h2, 2*h; 2*a21*h, 1+a22*h+a21*h2 ];
Bd = B(2)*Dinv * [2*h2;2*h];

end