function k = ssPlace( A, B, p )
% ssPlace compute the feedback gain of state-space model that yields a
% system with the desired eigenvalues (pole placement)
% 
% Syntax:
%   k = ssPlace( A, B, p )
% 
% Inputs:
%   A               system matrix (NxN array)
%   B               input matrix (Nx1 array)
%   p               desired poles (1xN array)
% 
% Outputs:
%   k               full state feedback gain (1xN array)
% 
% Example:
%   A = rand(4,4)
%   b = rand(4,1)
%   k = ssPlace( A, B, p )
% 
% See also:
%   place
% 
% Literature:
%   [1] Lunze, J. (2010). Regelungstechnik 1. Systemtheoretische
%       Grundlagen, Analyse und Entwurf einschleifiger Regelungen.
%       Springer, Berlin, Heidelberg.
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% eigenvalues to coefficients of characteristic polynomial (see [1], eq.
% (6.7))
a_d = poly(p);
a_d = fliplr(a_d(2:end));

% transformation to controllable canonical form (Regelungsnormalform)
[T_n,A_n,~] = ssCanonControl(A,B);

% open-loop coefficients of the characteristic polynomial (see [1], eq.
% (6.2))
a_n = -A_n(end,:);

% feedback gain in ncontrollable canonical form (see [1], eq. (6.8))
k_n = a_d - a_n;

% transform feedback gain back (see [1], eq. (6.12))
k = k_n * T_n;

end