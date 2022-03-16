function [acc] = trajSectionGetAcc(traj_section, vel, g, varargin)
% trajSectionGetAcc returns the acceleration from a trajectory
% section.
%
% Inputs:
%   traj_section   	trajectory section struct, see trajSectionInit        
%
%   vel             tangential velocity (scalar) in m/s
%
%   g               gravitational acceleration is approximately 9.81 m/s^2
%
%   t               dimensionless time paramter 
%                   (scalar), [0-1]
% Outputs:
%   acc             acceleration [ax; ay; az] in local geodetic system
%                   (3x1 vector), in m/s^2
%
% Syntax: 
%   [acc] = trajectoryGetAcc(traj_section, vel, g)
%   [acc] = trajectoryGetAcc(traj_section, vel, g, t) 
%
% See also: trajSectionInit, trajectoryGetPos, trajectoryGetVel,
%   trajectoryCreateFromWaypoints
%

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

% Calculate second derivative of path
[~,~,N,kappa,~] = trajSectionGetFrenetSerretWithGravity ...
    (traj_section, vel, g, t);

% Calculate path acceleration
acc = (kappa * N) * vel^2;

end