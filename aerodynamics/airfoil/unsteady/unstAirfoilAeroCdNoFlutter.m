function c_D = unstAirfoilAeroCdNoFlutter( c_D_st, c_L_unst, ...
    alpha, c_L_st )
% unstAirfoilAeroCdNoFlutter computes the unsteady drag coefficient.
%   This is an alternative implementation to unstAirfoilAeroCd. The
%   original function caused flutter in the VLM wing project. With this
%   implementation the flutter vanished.
% 
% Inputs:
%   c_D_st          steady drag coefficient
%   c_L_unst        unsteady lift coefficient (see unstAirfoilAeroFast)
%   alpha           angle of attack, in rad
%   c_L_st          static lift coefficient, in rad
% 
% Outputs:
%   c_D             unsteady drag coefficients
% 
% See also:
%   unstAirfoilAeroCd

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************


c_D = ( c_L_unst - c_L_st ).* sin( alpha ) + c_D_st;

end

