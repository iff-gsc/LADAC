function C_m_damp = simpleWingGetPitchDamping( wing, q_Aw, V_A ) %#codegen
% pitchDamping computes the pitching moment coefficient due to a pitch
% rate.
% 
% Inputs:
%   wing            simple wing struct (see loadSimpleWingParams)
%   q_Ab            pitching rate in wing frame, in rad/s
%   V_A             airspeed, in m/s
% 
% Outputs:
%   C_m_damp        pitching moment coefficient due to a pitch rate
% 
% Literature:
%   [1] Schlichting, H. & Truckenbrodt, E. (2001): Aerodynamik des
%       Flugzeuges - Teil 2, Springer.
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

C_m_dOmega = wing.C_m_dOmega_0;
c = wing.geometry.c;

% compute dimensionless pitching rate according to [1, p. 91]
Omega_y = q_Aw * c / max( V_A, 0.001 );

C_m_damp = C_m_dOmega * Omega_y;

end