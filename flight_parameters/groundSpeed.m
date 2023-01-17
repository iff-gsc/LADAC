% groundSpeed computes the absolute velocity of a point relative to 
%   the ground parallel to the horizontal plane.
% 
% Inputs:
%   V_Kg            flight path velocity represented in g frame (3xn vector
%                   where n is the number of flight path velocity vectors),
%                   in m/s
% 
% Outputs:
%   groundSpeed     ground speed (scalar or vector, depending on size
%                   of input), in m/s
% 
% See also: flightPathAzimuth
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

function groundSpeed = groundSpeed( V_Kg ) %#codegen

% determine number of input vectors
num_vectors = length( V_Kg(1,:) );
groundSpeed = zeros( 1, num_vectors );
% compute the flight-path azimuth according to [1, page 58]
for i = 1:num_vectors
    groundSpeed(i) = norm( V_Kg(1:2,i), 2 );
end

end