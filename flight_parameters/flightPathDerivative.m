% flightPathDerivative computes the time derivative of the flight-path
% velocity, flight-path angle and flight-path 
%   The time derivative of the flight path quantities (flight path
%   velocity, flight path angle and flight path azimuth) are obtained by
%   deriving their functional definition.
% 
% Literature:
%   [1] Brockhaus, R. et al. (2011): Flugregelung. 3rd ed. Springer-Verlag.
%   [2]	Beyer, Y., Kuzolap, A., Steen, M., Diekmann, J. H., & Fezans, N. 
%       (2018). Adaptive Nonlinear Flight Control of STOL-Aircraft Based on
%       Incremental Nonlinear Dynamic Inversion. In 2018 Modeling and 
%       Simulation Technologies Conference (AIAA 6.2018-3257).
% 
% Inputs:
%   V_Kb            three dimensional flight-path velocity in body-fixed
%                   frame (b), m/s
%   dot_V_Kb        three dimensional time derivative of the airspeed
%                   vector in body-fixed frame (b), m/s^2
%   M_kb            3x3 DCM from body-fixed frame (b) to flight-path frame
%                   (k)
%   M_gb            3x3 DCM from body-fixed frame (b) to NED frame (g)
%   omega_gb_b      three dimensional vector of the b frame relative to the
%                   g frame represented in b frame, rad/s
% 
% Outputs:
%   dot_V_K         scalar time derivative of the flight path velocity, m/s
%   dot_gamma       scalar time derivative of the flight path angle, rad
%   dot_chi         scalar time derivative of the flight path azimuth, rad
% 
% See also: aeroAngles
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

function [ dot_V_K, dot_gamma, dot_chi ] = flightPathDerivative( ...
    V_Kb, dot_V_Kb, M_kb, M_gb, omega_gb_b ) %#codegen

% transform the flight-path acceleration from b to k frame
dot_V_Kk = M_kb * dot_V_Kb;
% the first element is the absolute acceleration (because the other
% elements are zero), see [1, page 212]
dot_V_K = dot_V_Kk(1);

% transform the velocity from b to g frame
V_Kg = M_gb * V_Kb;
% compute the flight-path acceleration in g frame [1, page 213]
dot_V_Kg = M_gb * dot_V_Kb + cross(M_gb*omega_gb_b,V_Kg);

% The flight path azimuth is defined in [1, page 212] or indirectly through
% the definition of M_kg(gamma,chi) in [1, page 58]. The function is
% derived, the result is given in [2, page 10].
dot_chi = (dot_V_Kg(2)*V_Kg(1) - V_Kg(2)*dot_V_Kg(1)) / ...
    (V_Kg(2)^2 + V_Kg(1)^2);

% The flight path angle is defined in [1, page 212] or indirectly through
% the definition of M_kg(gamma,chi) in [1, page 58]. The function is
% derived, the result is given in [2, page 10].
dot_gamma = ( -dot_V_Kg(3) * (V_Kg(1)^2+V_Kg(2)^2) + ...
    V_Kg(3) * ( V_Kg(1)*dot_V_Kg(1) + V_Kg(2)*dot_V_Kg(2) ) ) / ...
    ( (V_Kg(1)^2+V_Kg(2)^2+V_Kg(2)^2) * sqrt(V_Kg(1)^2+V_Kg(2)^2) );

end