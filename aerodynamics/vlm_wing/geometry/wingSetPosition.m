function struct_ = wingSetPosition( struct_, state_vector, DOF, varargin )
% wingSetPosition sets the position matrix of characteristic points
% on the wing (e.g. control points)
% 
% Inputs:
%   struct_         Position struct containing the fields x, y, z, ...
%                   (see: wingSetGeometry)
%   xyz_matrix      Position (3xn or 4xn) matrix for each panel
% 
% Outputs:
%   struct_         Position struct containing the fields x, y, z, ...
%                   (see: wingSetGeometry)
% 

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

if ~isempty(varargin)
    abs = varargin{1};
else
    abs = false;
end   

if abs
    struct_.pos(1,:) = state_vector(1:DOF:end)';
    struct_.pos(2,:) = state_vector(2:DOF:end)';
    struct_.pos(3,:) = state_vector(3:DOF:end)';
    if DOF > 3
        struct_.local_incidence(:) = state_vector(4:DOF:end)';
    end
else
    struct_.pos(1,:) = struct_.pos(1,:) + state_vector(1:DOF:end)';
    struct_.pos(2,:) = struct_.pos(2,:) + state_vector(2:DOF:end)';
    struct_.pos(3,:) = struct_.pos(3,:) + state_vector(3:DOF:end)';
    if DOF > 3
        struct_.local_incidence(:) = struct_.local_incidence + state_vector(4:DOF:end)';
    end
end

end