% frictionModel computes friction forces depending on a normal force,
%   ground speed, flight path azimuth and parameters.
%   The friction model can be applied to multiple points. In that case
%   multiple normal forces, ground speeds and flight path azimuth must be
%   given and as a result multiple friction forces are computed.
% 
% Literature:
%   [1] Liu, Y. F. et al. (2015): Experimental comparison of five friction 
%       models on thesame test-bed of the micro stick-slip motion system. 
%       Mechanical Sciences, 6(1), 15-28.
% 
% Inputs:
%   F_N             normal force (1xn vector), in N
%   groundSpeed     ground speed (1xn vector), in m/s
%   chi             flight path azimut (1xn vector), in rad
%   mu              friction coefficient (scalar), in 1
%   k_V             scaling factor of tanh function for the computation of
%                   the friction slope vs. ground speed for small ground  
%                   speed (scalar), N/(m/s)
% 
% Outputs:
%   F_x_g           friction force in x direction of g frame
%   F_y_g           friction force in y direction of g frame
% 
% See also: groundNormalForce
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

function [ F_x_g, F_y_g ] = frictionModel( F_N, groundSpeed, chi, ...
    mu, k_V ) %#codegen

% compute maximum friction force
F_friction_max = mu * F_N;

% compute the friction slope vs. sliding speed referring to [1, page 17]
% (the tanh function is used to avoid shattering and slow simulation)
friction_force_vs_sliding_speed = tanh( k_V * groundSpeed );

% compute friction force
F_x_g = F_friction_max .* cos( chi ) .* friction_force_vs_sliding_speed;
F_y_g = F_friction_max .* sin( chi ) .* friction_force_vs_sliding_speed;

end