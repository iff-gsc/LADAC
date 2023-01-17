% oneMinusCosGustUds computes the Design Gust Velocity U_ds for a 1-cos
%   gust according to the requirements of Tilte 14, Code of Federal 
%   Regulations (14 CFR) 25.341, Gust and turbulence loads.
% 
% Inputs:
%   U_ref                   1-cos gust reference velocity, in m/s
%   gustGradDist            Gust Gradient Distance H \in [30ft,350ft], in m
% 
% Outputs:
%   U_ds                    Design Gust Velocity, in m/s. U_ds will be NaN
%                           if the input gustGradDist exceeds its bounds.
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

function U_ds = oneMinusCosGustUds( U_ref, gustGradDist, F_g ) %#codegen

% convert from m to ft
gustGradDist_ft = m2ft( gustGradDist );

% define maximum gust gradient distance, in ft [1, p. 5]
gustGradDistMax = 350;
% define minimum gust gradient distance, in ft [1, p. 5]
gustGradDistMin = 30;

% set Gust Gradient Distance to NaN if bounds are violated
gustGradDist_ft( gustGradDist_ft > gustGradDistMax ) = NaN;
gustGradDist_ft( gustGradDist_ft < gustGradDistMin ) = NaN;

% compute Design Gust Velocity, in ft [1, p. 6]
U_ds = U_ref * F_g * ( gustGradDist_ft / gustGradDistMax ).^(1/6);

end