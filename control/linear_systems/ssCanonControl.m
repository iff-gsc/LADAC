function [T_n,A_n,b_n] = ssCanonControl( A, b )
% ssCanonControl transform a state-space representation into the
% controllable canonical from
% 
% Syntax:
%   [T_n,A_n,b_n] = ssCanonControl( A, b )
%
% Inputs:
%   A           system matrix (NxN array)
%   b           input matrix (Nx1 array)
% 
% Outputs:
%   T_n         transformation matrix that satisfies A_n = T_n*A*inv(T_n)
%   A_n         system matrix in controllable canonical form (NxN array)
%   b_n         input matrix in controllable canonical form
% 
% Example:
%   A = rand(4,4)
%   b = rand(4,1)
%   [T_n,A_n,b_n] = ssCanonControl( A, b )
%   A_n == T_n*A*inv(T_n)
% 
% Literature:
%   [1] https://en.wikipedia.org/wiki/State-space_representation#Canonical_realizations
%   [2] Lunze, J. (2010). Regelungstechnik 1. Systemtheoretische
%       Grundlagen, Analyse und Entwurf einschleifiger Regelungen.
%       Springer, Berlin, Heidelberg.
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% [2], eq. (5.75)
S_s = ssControllability(A,b);

sys_order = size(A,2);

pick_last_row = zeros(1,sys_order);
pick_last_row(end) = 1;

% [2], eq. (5.76)
s_R = pick_last_row * inv(S_s);

% [2], eq. (5.77)
T_n = zeros(sys_order,sys_order);
for i=1:sys_order
    T_n(i,:) = s_R * A^(i-1);
end

% [2], below eq. (5.77)
A_n = T_n * A * inv(T_n);
b_n = T_n * b;

end