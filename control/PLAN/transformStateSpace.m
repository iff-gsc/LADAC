function [ A_t, B_t, C_t, D_t ] = transformStateSpace( A, B, C, D, T ) 

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2021 Alexander Kuzolap
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% nStatesNew = length( T(:,1) );
% nInputs = length( B(1,:) );
% nOutputs = length( C(:,1) );

% A_t = zeros( nStatesNew, nStatesNew );
% B_t = zeros( nStatesNew, nInputs );
% C_t = zeros( nOutputs, nStatesNew );
% D_t = zeros( nOutputs, nInputs );

A_t = T * A * pinv(T);
B_t = T * B;
C_t = C * pinv(T);
D_t = D;

end