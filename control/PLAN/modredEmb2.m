function [ A_red, B_red, C_red, D_red, condA_elim ] = modredEmb2( ...
    A, B, C, D, elim )
%#codegen
% modredEmb is similar to the MATLAB function modred. However, modred does
%   not work in embedded code. modredEmb works with embedded code.
%   Passthrough inputs must have correct dimension but arbitrary values.
% 
% Inputs:
%   A           system matrix (nxn)
%   B           input matrix (nxp)
%   C           output matrix (qxn)
%   D           feedthrough matrix (qxp)
%   elim        logical vector (nx1), true for states to be eliminated
% 
% Outputs:
%   A_red       reduced system matrix (n_red x n_red)
%   B_red       reduced input matrix (n_red x p)
%   C_red       reduced output matrix (q x n_red)
%   D_red       reduced feedthrough matrix (qxp)
%   condAelim   condition number of to be inverted matrix, for monitoring
% 
% See also: modred
% 

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2021 Alexander Kuzolap
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

    nelim = not( elim );
    
    nStates_red = sum( nelim );
    nInputs = length( B(1,:) );
    nOutputs = length( C(:,1) );
    
    A_red = zeros( nStates_red, nStates_red );
    B_red = zeros(  nStates_red, nInputs );
    C_red = zeros( nOutputs, nStates_red );
    D_red = zeros( nOutputs, nInputs );

    A11 = A( nelim, nelim );
    A22 = A( elim, elim );
    A12 = A( nelim, elim );
    A21 = A( elim, nelim );
    B1 = B( nelim, : );
    B2 = B( elim, : );
    C1 = C( :, nelim );
    C2 = C( :, elim );

    condA_elim = cond( A22 );
    if ( ~isinf( condA_elim ) && condA_elim < 5.E04 )
%         inv_A22 = inv( A22 );
        inv_A22_times_A21 = A22 \ A21;
        inv_A22_times_B2 = A22 \ B2;
        
        A_red = A11 - A12 * inv_A22_times_A21;
        B_red = B1 - A12*inv_A22_times_B2;
        C_red = C1 - C2*inv_A22_times_A21;
        D_red = D - C2*inv_A22_times_B2;
    end
end
