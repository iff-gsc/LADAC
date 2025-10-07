function eA = expm2( A )
% expm2 is a faster implementation of expm(A) for a matrix of the following
% format: A = [ 0, T; a21, a22 ] with T > 0
% 
% Inputs:
%   A           Matrix of the format: [ 0, T; a21, a22 ] with T > 0
% 
% Outputs:
%   eA          Matrix exponential expm(A) (2x2 matrix)

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2025 Yannic Beyer
%   Copyright (C) 2025 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

if ~( A(1,1)==0 && A(1,2)>0 && isequal(size(A),[2,2]))
    error('The given matrix does not match the required format.');
end

T = A(1,2);
A = A/T;
a21 = A(2,1);
a22 = A(2,2);

mu = a22/2;
delta = mu^2 + a21;

exp_muT = exp(mu*T);

% Case 1: underdamped (delta<0)
if delta < 0
    omega = sqrt(-delta);
    cos_w = cos(omega*T);
    sin_w = sin(omega*T);
    eA = exp_muT * ...
        [ cos_w-mu/omega*sin_w, 1/omega*sin_w; ...
        a21/omega*sin_w, cos_w+mu/omega*sin_w ];
% Case 2: overdamped (delta>0)
elseif delta > 0
    omega = sqrt(delta);
    cos_w = cosh(omega*T);
    sin_w = sinh(omega*T);
    eA = exp_muT * ...
        [ cos_w-mu/omega*sin_w, 1/omega*sin_w; ...
        a21/omega*sin_w, cos_w+mu/omega*sin_w ];
% Case 3: critically damped (delta=0)
else
    eA = exp_muT * [ 1-mu*T, T; a21*T, 1+mu*T ];
end

end