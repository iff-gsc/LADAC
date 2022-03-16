function [n] = trajSectionGetLoadFactor(acc_vec)
% trajSectionGetLoadFactor calculates the load factor of acceleration.
%   trajSectionGetLoadFactor returns the load factor for a given
%   acceleration vector acc relative to gravitational earth acceleration 
%   that is approximately 9.81 m/s^2.
%   
% Inputs:
%  acc_vec          acceleration [ax; ay; az] in local geodetic system
%                   (3x1 vector), in m/s^2    
%
% Outputs:
%   n               load-factor relative to gravitational acceleration
%                   (scalar), dimensionsless
%
% Syntax:
%   n = trajSectionGetVel(acc_vec) 
%
% See also: trajSectionGetAcc
%

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Fabian Guecker
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% g : gravitational acceleration (approximately 9.81 m/s^2)
g = 9.81;

% Calculate the load-factor
n = norm(acc_vec) / g;

end