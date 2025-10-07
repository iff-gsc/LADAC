function invA = inv2( A )
% inv2 is a faster implementation of inv(A) for a 2x2 matrix. The inverse
% of a singular matrix will be Zero instead of Inf.
% 
% Inputs:
%   A           2x2 matrix
% 
% Outputs:
%   inv2        Inverse of matrix A (2x2 matrix)

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2025 Yannic Beyer
%   Copyright (C) 2025 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

invA = divideFinite(1,(A(1,1)*A(2,2)-A(1,2)*A(2,1))) * ...
    [A(2,2),-A(1,2); -A(2,1),A(1,1)];

end