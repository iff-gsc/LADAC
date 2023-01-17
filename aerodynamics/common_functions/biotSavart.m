function AIC = biotSavart( v_inf_local, r_vortex, r_cntrl )
%biotSavart computes the aerodynamic influence coefficients.
% 
% Inputs:
%   v_inf_local     freestream local airspeed at each control point (3xN_c
%                   array, where N_c is the number of control points), 
%                   in m/s
%   r_vortex        position vectors of the edges of the horseshoe vortices
%                   where it is assumed that r_vortex(:,j) and
%                   r_vortex(:,j+1) are the edges of one horseshoe vortex
%                   (3x(N_v+1) array, where N_v is the number of horseshoe
%                   vortices), in m
%   r_cntrl         position vectors of the control points (3xN_c array),
%                   in m
% 
% Outputs:
%   AIC             aerodynamic influence coefficients (3 x N_c x N_v
%                   array), in 1/m
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% init output
N_c     = size( r_cntrl, 2 );
N_v     = size( r_vortex, 2 ) - 1;

v_inf_local_mat = repmat( permute(v_inf_local,[1,3,2]), 1, N_c, 1 );

% define horseshoe vortices to be aligned with the c/4 line and the
% freestream
% r_cntrl_mat = repmat(r_cntrl,1,1,N_v);
% r_vortex_mat = repmat(r_vortex,1,1,N_v);
r_1 = repmat(r_cntrl,1,1,N_v) - repmat( permute(r_vortex(:,1:end-1), [1,3,2] ), 1,N_c,1);
r_2 = repmat(r_cntrl,1,1,N_v) - repmat( permute(r_vortex(:,2:end), [1,3,2] ), 1,N_c,1);

% length of vectors
r_1_mag = vecnorm( r_1, 2, 1 );
r_2_mag = vecnorm( r_2, 2, 1 );

% Biot-Savart for trailing vortices and bound vortices [1], eq. (6)
AIC = 1/(4*pi) * (...
    cross( v_inf_local_mat, r_2, 1 ) ./ repmat( r_2_mag .* ( r_2_mag - dot( v_inf_local_mat, r_2, 1 ) ), 3, 1, 1 ) ...
    +  cross( r_1, r_2, 1 ) .* repmat( ( r_1_mag + r_2_mag ) ./ ( r_1_mag .* r_2_mag .* ( r_1_mag.*r_2_mag + dot( r_1, r_2, 1 ) ) ), 3, 1, 1 ) ...
    - cross( v_inf_local_mat, r_1, 1 ) ./ repmat( r_1_mag .* ( r_1_mag - dot( v_inf_local_mat, r_1, 1 ) ), 3, 1, 1 ) ...
    );

end

