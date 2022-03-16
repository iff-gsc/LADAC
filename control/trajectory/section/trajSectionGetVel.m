function [vel_vec] = trajSectionGetVel(traj_section, vel, varargin)
% trajSectionGetVel returns the velocity vector for scalar velocity 
%   The function returns a velocity vector in geodetic coordinate system
%   from a given trajectory section and scalar velocity velo.
%   The given velocity vel is assumed to be tanget to the trajectory.
%   The result is a velocity vector in geodetic coordinate system that
%   is tangent to the trajectory with the length of the given scalar
%   velocity vel.
%   
% Inputs:
%   traj_section 	trajectory section struct, see trajSectionInit       
%
%   vel             trajectory tangent velocity
%                   (scalar), in m/s
%
%   t               dimensionless time paramter 
%                   (scalar), [0-1]
%
% Outputs:
%   vel         velocity [vx; vy; vz] in local geodetic coordinate system
%               (3x1 vector), in m/s
%
% Syntax:
%   vel_vec = trajSectionGetVel(traj_section, vel) 
%   vel_vec = trajSectionGetVel(traj_section, vel, t) 
%
% See also: trajSectionInit, trajSectionGetPos, trajSectionGetAcc

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Fabian Guecker
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

t = 0;

if isempty(varargin)
    t(:) = traj_section.t;
else
    t(:) = varargin{1};
end   

% Calculate first derivative of path
[firstDerivative] = trajSectionGetDerivatives(traj_section, t);
  
% Calculate unit-length derivative vector
norm_firstDerivative = firstDerivative / max(eps, norm(firstDerivative));

% Calculate velocity in local geodetic coordinate system
vel_vec = vel * norm_firstDerivative;

end