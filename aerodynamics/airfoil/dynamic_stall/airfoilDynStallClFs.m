function c_L_fs = airfoilDynStallClFs( c_L_st, c_L_alpha, alpha0, alpha, f_st ) %#codegen
% airfoilDynStallClFs computes the lift coefficient for fully separated
% flow according to [1], eq. (18) and (19).
%   This function can be run for N airfoils/conditions at once.
% 
% Inputs:
%   c_L_st          values of the static lift curve (Nx1 array)
%   c_L_alpha       (maximum) lift curve slope (Nx1 array), in 1/[alpha]
%   alpha           angle of attack (Nx1 array) in deg or rad
%   alpha0          zero lift angle of attack (Nx1 array) in [alpha]
%   f_st            separation point (Nx1 array), dimensionless
% 
% Outputs:
%   c_L_fs         	separation point (Nx1 array), dimensionless
% 
% Literature:
%   [1] https://backend.orbit.dtu.dk/ws/portalfiles/portal/7711084/ris_r_1354.pdf
% 
% See also: airfoilDynStallFst, airfoilDynStall
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

Delta_alpha = alpha-alpha0;
idx = abs(Delta_alpha) <= 0.5;
c_L_fs = zeros(length(c_L_st),1);
is_attached = f_st > 1 - 1e-5;
c_L_fs(~is_attached) = ( c_L_st(~is_attached) - c_L_alpha(~is_attached) .* Delta_alpha(~is_attached) .*cos(pi/2/90*Delta_alpha(~is_attached)) .* f_st(~is_attached) ) ./ ( 1 - f_st(~is_attached) );
c_L_fs(is_attached) = c_L_st(is_attached)/2;

c_L_fs(idx) = c_L_st(idx)/2;

end