% oneMinusCosGustUds computes the Reference Gust Velocity U_ref for a 1-cos
%   gust according to the requirements of Tilte 14, Code of Federal 
%   Regulations (14 CFR) 25.341, Gust and turbulence loads.
% 
% Inputs:
%   h                       altitude, in m
% 
% Outputs:
%   U_ref                   Reference Gust Velocity, in m/s.
% 
% Literature:
%   https://www.faa.gov/documentLibrary/media/Advisory_Circular/AC_25_341-1.pdf
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

function U_ref = oneMinusCosGustUref( h ) %#codegen

% convert from m to ft
h_ft = m2ft( h );

% get air density
atmosphere = isAtmosphere( h );
rho = atmosphere.rho;

% set air density at mean sea level
atmosphere = isAtmosphere( 0 );
rho_0 = atmosphere.rho;

% compute 1-cos gust reference velocity (convert from EAS to TAS), in ft/s
% [1, p. 5]
if h_ft < 15000
    U_ref_ft = sqrtReal(rho_0/rho)*( 56 - 12 * h_ft / 15000 ); 
else
    U_ref_ft = sqrtReal(rho_0/rho)*( 44 - 18 * ( h_ft - 15000 ) / 35000 );
end

U_ref = ft2m( U_ref_ft );

end