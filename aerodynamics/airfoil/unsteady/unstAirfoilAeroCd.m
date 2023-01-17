function c_D = unstAirfoilAeroCd( c_D_st, c_L_unst, ...
    alpha, alpha_E )
% unstAirfoilAeroCd computes the unsteady drag coefficient.
%   The implementation differs from [1] and [2] but is very similar to [2].
%   The dependency on the trailing edge separation point was dropped
%   because the impact was very small.
%   c_D_st instead of c_D_st_E was used because the result was better.
% 
% Inputs:
%   c_D_st          steady drag coefficient
%   c_L_unst        unsteady lift coefficient (see unstAirfoilAeroFast)
%   alpha           angle of attack, in rad
%   alpha_E         unsteady/effective angle of attack (see
%                   unstAirfoilAeroFast), in rad
% 
% Outputs:
%   c_D             unsteady drag coefficients
% 
% See also:
%   unstAirfoilAeroFast
% 
% Literature:
%   [1] Leishman, J. G. (1988). Two-dimensional model for airfoil unsteady
%       drag below stall. Journal of Aircraft, 25(7), 665-666.
%   [2] https://backend.orbit.dtu.dk/ws/portalfiles/portal/7711084/ris_r_1354.pdf
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************


c_D = c_L_unst .* sin( alpha - alpha_E ) + c_D_st;

end

