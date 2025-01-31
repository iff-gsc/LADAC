function f_st = airfoilDynStallFst( c_L_st, c_L_alpha, Delta_alpha ) %#codegen
% airfoilDynStallFst computes the separation point according to [1], eq. (15)
%   This function can be run for N airfoils/conditions at once.
% 
% Inputs:
%   c_L_st          values of the static lift curve (1xN array)
%   c_L_alpha       (maximum) lift curve slope (1xN array), in 1/deg
%   Delta_alpha     angle of attack minus zero lift angle of attack (1xN
%                   array), in deg
% 
% Outputs:
%   f_st            separation point (1xN array), dimensionless
% 
% Literature:
%   [1] https://backend.orbit.dtu.dk/ws/portalfiles/portal/7711084/ris_r_1354.pdf
% 
% See also: airfoilDynStallFst, airfoilDynStallClFs
% 

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

abs_Delta_alpha = abs(Delta_alpha);

f_st = powerFast( 2*(sqrtReal( c_L_st ./ ( c_L_alpha.*Delta_alpha.*cos(pi/2/90*Delta_alpha) ) ) ) - 1, 2 );

f_st(f_st>1) = 1;
f_st(abs_Delta_alpha > 70) = 0;
% avoid division by zero: set f_st to 1
f_st(abs_Delta_alpha < 0.5) = 1;

end