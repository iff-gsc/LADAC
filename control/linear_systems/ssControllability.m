function [S_s,is_controllable] = ssControllability( A, b )
% ssControllability compute the controllability matrix of a state-space
% representation
% 
% Syntax:
%   [S_s,is_controllable] = ssControllability( A, b )
% 
% Inputs:
%   A               system matrix (NxN array)
%   b               input matrix (Nx1 array)
% 
% Outputs:
%   S_s             controllability matrix (NxN array)
%   is_controllable boolean whether the system is controllable
% 
% Example:
%   A = rand(4,4)
%   b = rand(4,1)
%   [S_s,is_controllable] = ssControllability( A, b )
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

sys_order = size(A,2);

% [2], eq. (5.75)
S_s = zeros(sys_order,sys_order);
for i=1:sys_order
    S_s(:,i) = A^(i-1) * b;
end

is_controllable = rank(S_s) == sys_order;

end