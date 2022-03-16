function [ R_Aa, Q_Aa ] = coeffs2ForcesMoments( C_lmn_i_a, C_XYZ_i_a, ...
    q, S_i, b, c ) %#codegen
% coeffs2ForcesMoments computes the aerodynamic force and moment vector
%   from aerodynamic coefficients for the simple wing model in an arbitrary
%   frame (a). Usually the wing frame is used.
%   However, note that all input and output vectors must be represented in
%   the same frame.
% 
% Inputs:
%   C_lmn_i_a       moment coefficients (3xn) vector for n points in
%                   arbitrary frame, in 1
%   C_XYZ_i_a       force coefficients (3xn) vector for n points in
%                   arbitrary frame, in 1
%   q               dynamic pressure (1xn) vector, in Pa
%   S_i             reference surface (1xn) vector, in m^2
%   b               wing span (scalar), in m
%   c               mean aerodynamic chord (scalar), in m
% 
% Outputs:
%   R_Aa            Aerodynamic force (3x1) vector in arbitrary frame, in N
%   Q_Aa            Aerodynamic moment (3x1) vector in arbitrary frame, in
%                   Nm
% 
% Literature:
%   [1] Schlichting, H. & Truckenbrodt, E. (2001): Aerodynamik des
%       Flugzeuges - Teil 2, Springer.
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% determine number of points
n = length( C_XYZ_i_a(1,:) );

q_rep = repmat( q, 3, 1 );

R_Aa = sum( C_XYZ_i_a .* q_rep * S_i, 2 );

ref_length_lmn = [ b/2; c; b/2 ];

Q_Aa = sum( C_lmn_i_a .* q_rep .* repmat( ref_length_lmn, 1, n ) * S_i, 2 );

end