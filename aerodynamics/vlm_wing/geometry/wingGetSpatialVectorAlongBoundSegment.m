function dl = wingGetSpatialVectorAlongBoundSegment( vortex, varargin )
% getSpatialVectorAlongBoundSegment computes the spatial vector along the
%   bound segment dl according to [1].
% 
% Inputs:
%   vortex              struct containing vgeometric vortex parameters as
%                       defined by setGeometry.m
% 
% Outputs:
%   dl                  spatial vector (3xn) along the bound segment
% 
% Literature:
%   [1] Phillips, W. F., & Snyder, D. O. (2000). Modern adaption of
%       Prandtl's classic lifting-line theory. Jounal of Aircraft, 37(4),
%       662-670.
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% compute dl according to [1] (no equation provided)

if isempty(varargin)
    dl = diff( vortex.pos, 1, 2 );
else
    idx = varargin{1};
    dl = diff( vortex.pos(:,idx), 1, 2 );
end

end