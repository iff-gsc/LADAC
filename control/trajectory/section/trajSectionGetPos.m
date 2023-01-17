function [pos] = trajSectionGetPos(traj_section, varargin)
% trajSectionGetPos returns the position from a trajectory
%   The function returns the positon vector from a given trajectory
%   specified by the the dimensionless time parameter t from 
%   the traj_section struct or can be given via varargin as optional
%   parameter in range [0-1].
%
% Inputs:
%   traj_section 	trajectory section struct, see trajSectionInit         
%
%   t               dimensionless time parameter, range [0-1]
%
% Outputs:
%   pos             position [x; y; z] in local geodetic coordinate system
%                   (3x1 vector), in m
%
% Syntax:
%   [pos] = trajSectionGetPos(traj_section) 
%   [pos] = trajSectionGetPos(traj_section, t) 
%
% See also: trajSectionInit, trajSectionGetVel, trajSectionGetAcc
%

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Fabian Guecker
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************
t = 0;

if isempty(varargin)
    t(:) = traj_section.t;
else
    t(:) = real(varargin{1});
end    

% Calculate x-positon with t
px = polyVal(traj_section.pos_x, t);

% Calculate y-positon with t
py = polyVal(traj_section.pos_y, t);

% Calculate z-positon with t
pz = polyVal(traj_section.pos_z, t);

% return position vector
pos = [px; py; pz];

end