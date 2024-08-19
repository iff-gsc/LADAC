function [ xyz_matrix, struct_ ] = wingGetPositionMatrix( struct_, varargin )
% wingGetPositionMatrix gets the position matrix of characteristic points
% on the wing (e.g. control points)
% 
% Inputs:
%   struct_         Position struct containing the fields x, y, z
%                   (see: wingSetGeometry)
% 
% Outputs:
%   xyz_matrix      Position (3xn) matrix for each panel
% 
% Example:
%   xyz_matrix = wingGetPositionMatrix( wing.geometry.vortex )
% 

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

if isempty(varargin)
    xyz_matrix = [ struct_.x; struct_.y; struct_.z ];
else
    idx = varargin{1};
    xyz_matrix = [ struct_.x(idx); struct_.y(idx); struct_.z(idx) ];
end

end