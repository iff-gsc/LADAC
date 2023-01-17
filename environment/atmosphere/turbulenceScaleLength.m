function [ L_u, L_v, L_w ] = turbulenceScaleLength( h ) %#codegen
% turbulenceScaleLength computes the the wind turbulence scale lengths
%   for a Dryden spectrum according to the military references MIL-F-8785C.
% 
% Inputs:
% 	h               altitude (scalar), in m
% 
% Outputs:
%   L               Root-mean-square intensity of wind velocity, in m/s
% 
% Literature:
%   [1] Gage, S. (2003): Creating a Unified Graphical Wind Turbulence Model
%       from Multiple Specifications. In: AIAA Modeling and Simulation
%       Technologies Conference and Exhibit. AIAA 2003-5529. Austin, Texas.
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% feet to meter
h_ft = m2ft( h );

% compute Dryden turbulence parameters
if h_ft < 1000
    % compute RMS turbulence intensity for low altitude according to [1, p.
    % 5]
    L_u = ft2m( h_ft / ( 0.177+0.000823*h_ft ).^1.2 );
    L_v = L_u;
    L_w = h;
elseif h_ft > 2000
    % compute RMS turbulence intensity for medium/high altitude according
    % to [1, 6]
    L_u = ft2m( 1750 );
    L_v = L_u;
    L_w = L_u;
else
    L_1000 = ft2m( 1000 );
    L_2000 = ft2m( 1750 );
    L_u = interp1( [ 1000, 2000 ], [ L_1000, L_2000 ], h_ft );
    L_v = L_u;
    L_w = L_u;
end

end