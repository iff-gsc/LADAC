function [ alpha, beta ] = aeroAngles( V_Ab ) %#codegen
% aeroAngles computes the angle of attack and sideslip angle
%   The angle of attack and the sideslip angle are computed according to
%   the ISO 1151 (or LN9300).
% 
% Literature:
% [1]   Stevens, B. L. et al. (2016): Aircraft Control and Simulation.
%       Dynamics, Control Design, and Autonomous Systems. 3rd ed. Wiley.
% 
% Inputs:
%   V_Ab            a (3xn) matrix with the components of the airspeed
%                   vector in body-fixed frame (b), where n airspeed
%                   vectors can be processed at once
% 
% Outputs:
%   alpha           a (1xn) vector with n angles of attack for n airspeed
%                   vectors
%   beta            a (1xn) vector with n sideslip angles for n airspeed
%                   vectors
% 
% See also: dcmBaFromAeroAngles

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% compute the angle of attack according to [1, page 78]
alpha = atan2( V_Ab(3,:), V_Ab(1,:) );

% compute the sideslip angle according to [1, page 78]
beta = asinReal( divideFinite( V_Ab(2,:), vecnorm( V_Ab, 2, 1 ) ) );

end