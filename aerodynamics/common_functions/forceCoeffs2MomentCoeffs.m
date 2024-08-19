function [ C_lmn_i_w ] = forceCoeffs2MomentCoeffs( xyz_cp_w, C_XYZ_w, c, b ) %#codegen
% forceCoeffs2MomentCoeffs computes the moments coefficients from the force
%   coefficients at given center of pressures (cp).
% 
% Inputs:
%   xyz_cp_w    center of pressure 3xn matrix in wing frame, in m
%   C_XYZ_w     force coefficients 3xn matrix in wing frame, in 1
%   c           reference mean chord, in m
%   b           reference span, in m
% 
% Outputs:
%   C_lmn_i_w   moments coefficients 3xn matrix in wing frame, in m
% 

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

len = length( xyz_cp_w(1,:) );
C_lmn_i_w = cross( xyz_cp_w, C_XYZ_w ) ./ repmat( [ b/2; c; b/2 ], 1, len );

end