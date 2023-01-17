function external = fuselageExternalInit( n_segments )
%fuselageExternalInit define and initialize external struct for fuselage
% struct
%   The external struct contains information about external states such as
%   wind velocity and atmospheric parameters.
% 
% Inputs:
%   n_segments      number of segments for the discretized aerodynamics
%                   model (scalar)
% 
% Outputs:
%   external        external struct as defined by this function
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% wind velocity at each segment border point in body frame, in m/s
external.V_Wb = zeros( 3, n_segments+1 );
% wind acceleration at each segment border point in body frame, in m/s^2
external.V_Wb_dt = zeros( 3, n_segments+1 );
% atmospheric parameters at the current altitude (see isAtmosphere)
external.atmosphere = isAtmosphere(0);

end