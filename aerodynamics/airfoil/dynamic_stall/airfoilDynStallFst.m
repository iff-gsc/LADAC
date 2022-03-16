function f_st = airfoilDynStallFst( c_L_st, c_L_alpha, Delta_alpha ) %#codegen
% airfoilDynStallFst computes the separation point according to [1], eq. (15)
%   This function can be run for N airfoils/conditions at once.
% 
% Inputs:
%   c_L_st          values of the static lift curve (Nx1 array)
%   c_L_alpha       (maximum) lift curve slope (Nx1 array), in 1/deg
%   Delta_alpha     angle of attack minus zero lift angle of attack (Nx1
%                   array), in deg
% 
% Outputs:
%   f_st            separation point (Nx1 array), dimensionless
% 
% Literature:
%   [1] https://backend.orbit.dtu.dk/ws/portalfiles/portal/7711084/ris_r_1354.pdf
% 
% See also: airfoilDynStallFst, airfoilDynStallClFs
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% avoid division by zero: inices
idx = abs(Delta_alpha) <= 0.5;

f_st = min( 1, ( 2*(sqrtReal( max( 0, c_L_st(:) ./ ( c_L_alpha.*Delta_alpha(:).*cos(pi/2/90*Delta_alpha(:)) ) ) ) ) - 1 ).^2 );
        
f_st(abs(Delta_alpha) > 70) = 0;

% avoid division by zero: set f_st to 1
f_st(idx) = 1;

end