function [ sigma_u, sigma_v, sigma_w ] = turbulenceIntensityRMS( h, s_exp, V_wind_6m ) %#codegen
% turbulenceIntensityRMS computes the root-mean-square turbulence intensity
%   according to the military references (MIL-HDBK-1797, MIL-F-8785C).
% 
% Inputs:
% 	h               altitude (scalar), in m
%   s_exp           severity exponent vector (e.g. -2=light, -3=moderate, 
%                   -5=severe), see [1, p 6], in 1
%   V_wind_6m       wind velocity 20 feet above the ground (15knots=
%                   7.716m/s=light, 30knots=15.432m/s=moderate, 45knots=
%                   23.148m/s=severe), in m/s
% 
% Outputs:
%   sigma           Root-mean-square intensity of wind velocity, in m/s
% 
% Literature:
%   [1] Gage, S. (2003): Creating a Unified Graphical Wind Turbulence Model
%       from Multiple Specifications. In: AIAA Modeling and Simulation
%       Technologies Conference and Exhibit. AIAA 2003-5529. Austin, Texas.
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% feet to meter
h_ft = m2ft( h );

% definition of severity exponent vector (data from Matlab Aerospace
% Blockset)
s_exp_vec = -[ -0.6990 -1 -2 -3 -4 -5 -6 ];

% definition of altitude vector (data from Matlab Aerospace Blockset)
h_vec = ft2m( [ 500 1750 3750 7500 15000 25000 35000 45000 55000 65000 ...
    75000 80000 ]' );

% definition of intensity map depending on altitude and severity exponent
% (data from Matlab Aerospace Blockset)
sigma_mat = ft2m( [ ...
    3.2 4.2 6.6 8.6 11.8 15.6 18.7; ...
    2.2 3.6 6.9 9.6 13 17.6 21.5; ...
    1.5 3.3 7.4 10.6 16 23 28.4; ...
    0 1.6 6.7 10.1 15.1 23.6 30.2; ...
    0 0 4.6 8 11.6 22.1 30.7; ...
    0 0 2.7 6.6 9.7 20 31; ...
    0 0 0.4 5 8.1 16 25.2; ...
    0 0 0 4.2 8.2 15.1 23.1; ...
    0 0 0 2.7 7.9 12.1 17.5; ...
    0 0 0 0 4.9 7.9 10.7; ...
    0 0 0 0 3.2 6.2 8.4; ...
    0 0 0 0 2.1 5.1 7.2 ...
    ] );

[X,Y] = meshgrid(s_exp_vec,h_vec);

% compute Dryden turbulence parameters
if h_ft < 1000
    % compute RMS turbulence intensity for low altitude according to [1, p.
    % 5]
	sigma_w = 0.1 * V_wind_6m;
	sigma_u = 1 ./ ( 0.177+0.000823*h_ft ).^0.4 * sigma_w;
    sigma_v = sigma_u;
elseif h_ft > 2000
    % compute RMS turbulence intensity for medium/high altitude according
    % to [1, 6]

    sigma_u = interp2( X, Y, sigma_mat, -s_exp, h );

    sigma_v = sigma_u;
    sigma_w = sigma_v;
    
else
    % interpolate linearly between both regions [1, p. 6]
    sigma_1000 = 0.1 * V_wind_6m;
    sigma_2000 = interp2( X, Y, sigma_mat, -s_exp, ft2m(2000) );
    sigma_u = interp1( [ 1000, 2000 ], [ sigma_1000, sigma_2000 ], h_ft );
    sigma_v = sigma_u;
    sigma_w = sigma_u;    
end

end