function [error, idx] = trajGetError(traj, t, pos_array, sec_idx)
% trajGetError computes the minimum tracking error.
%   The tracking error is the minimum euclidean distance between the 
%   vehicle and the trajectory. The function returns the minimum euclidean
%   distance between the trajectory and the positions given in pos_array.
%   Likewise, it provides the corresponding index idx.
%
% Inputs:
%   traj            trajectory struct, see trajInit
%
%   t               dimensionless time parameter of the current section
%                	(scalar), [0-1]
%
%   pos_array       position vector with N positions
%                   (3xN vector), in m
%
%   sec_idx         section index for each position
%                   (1xN vector), dimensionless
%
% Outputs:
%   error           the minimum euclidean distance between the given
%                   positions and the and the trajectory, 
%                   (scalar), in m
%
%   idx             index that corresponds to the minimum distance of
%                   pos_array
%                   (scalar), in m
%
% Syntax:
%   [error, idx] = trajGetError(traj, t, pos_array, sec_idx)
%
% See also: trajInit, trajGetMatch, trajCreateFromWaypoints
%

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Fabian Guecker
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

m = size(t,1);

Errors = zeros(1,m);
pos = zeros(3,m);

for j=1:m

traj_section = trajGetSection(traj, sec_idx(j));   
pos(:,j) = trajSectionGetPos(traj_section,t(j));
% compute the tracking error
Errors(j) = norm(pos(:,j) - pos_array);

end
% the minimum euclidean distance between the vehicle and the trajectory
[error, idx]= min(Errors);

end