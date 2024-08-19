% oneMinusCosGustFg0 computes the Flight Profile Alleviation Factor for 
%   a 1-cos gust according to the requirements of Tilte 14, Code of Federal 
%   Regulations (14 CFR) 25.341, Gust and turbulence loads.
% 
% Inputs:
%   F_g0                    Flight Profile Alleviation Factor at sea level,
%                           in 1
%   alt_mo                  Maximum operating altitude, in m
%   alt                     Altitude, in m
% 
% Outputs:
%   F_g                     Flight profile alleviation factor, in 1
% 
% Literature:
%   https://www.law.cornell.edu/cfr/text/14/25.341#a_6
% 

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

function F_g = oneMinusCosGustFg( F_g0, alt_mo, alt ) %#codegen

F_gMax = 1;
alt_0 = 0;

alt_vector = [ alt_0, alt_mo ];
F_g_vector = [ F_g0, F_gMax ];

F_g = interp1( alt_vector, F_g_vector, alt );

end