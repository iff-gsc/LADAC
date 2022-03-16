% flightPathAzimuth computes the flight-path azimuth chi based on
% conventions of ISO 1151 (or LN9300).
%   The flight path azimuth is computed by the flight path velocity vector.
%   It is possible to compute multiple flight path azimuths if the input is
%   a matrix of concentrated flight path velocity vectors.
% 
% Literature:
%   [1] Brockhaus, R. et al. (2011): Flugregelung. 3rd ed. Springer-Verlag.
% 
% Inputs:
%   V_Kg            flight path velocity represented in g frame (3xn vector
%                   where n is the number of flight path velocity vectors),
%                   in m/s
% 
% Outputs:
%   chi             flight path azimut (scalar or vector, depending on size
%                   of input), in rad
% 
% See also: aeroAngles
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

function chi = flightPathAzimuth( V_Kg ) %#codegen

% determine number of input vectors
num_vectors = length( V_Kg(1,:) );
chi = zeros( 1, num_vectors );
% compute the flight-path azimuth according to [1, page 58]
for i = 1:num_vectors
    chi(i) = atan2( V_Kg(2,i), V_Kg(1,i) );
end

end