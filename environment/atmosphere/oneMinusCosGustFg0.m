% oneMinusCosGustFg0 computes the Flight Profile Alleviation Factor at 
%   sea level for a 1-cos gust according to the requirements of Tilte 14, 
%   Code of Federal Regulations (14 CFR) 25.341, Gust and turbulence loads.
% 
% Inputs:
%   m_maxLand               Maximum landing mass, in (same unit as other
%                           mass inputs)
%   m_maxTO                 Maximum take-off mass, in (same unit as other
%                           mass inputs)
%   m_maxZeroFuel           Maximum zero fuel mass, in (same unit as other
%                           mass inputs)
%   alt_mo                  Maximum operating altitude, in m
% 
% Outputs:
%   F_g0                    Flight profile alleviation factor at sea level,
%                           in 1
% 
% Literature:
%   https://www.law.cornell.edu/cfr/text/14/25.341#a_6
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

function F_g0 = oneMinusCosGustFg0( m_maxLand, m_maxTO, m_maxZeroFuel, alt_mo ) %#codegen

R_1 = m_maxLand / m_maxTO;
R_2 = m_maxZeroFuel / m_maxTO;

F_gm = sqrt( R_2 * tan( pi * R_1 / 4 ) );

Z_mo_ft = m2ft( alt_mo );
F_gz = 1 - Z_mo_ft / 250000;

F_g0 = 0.5 * ( F_gz + F_gm );

end