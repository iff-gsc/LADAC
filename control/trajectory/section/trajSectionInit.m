function traj_section = trajSectionInit(varargin)
% trajSectionInit defines a trajectory section struct.
%   The function initializes a struct with the coefficents of a cubic
%   spline for all three dimensions.
% 
% Inputs:
%   degree          Degree of a polynomials in the trajectory section
%                   (optional, default = 3)
%
% Outputs:
%   traj_section    trajectory section struct where the position
%                   (pos_x, pos_y, pos_z) is a vector of length n+1 whose
%                   elements are the coefficients (in descending powers)
%                   of an nth-degree polynomial.
%                   t is the dimensionless run parameter which lies
%                   in the range 0-1. For example:
%                       x(t) = 3t^2 + 2^t + 8 
%                       pos_x = [3 2 8]
%
%                   The target velocity is in the same form as
%                   described above.
%
% Syntax: 
%   traj_section = trajSectionInit()
%
% See also: trajSectionSet, trajSectionGetPos

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Fabian Guecker
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% Set default degree of the polynominal
degree = 3;

if (~isempty(varargin))
    degree = max(real(varargin{1}), 2);
end   

traj_section = struct( ...
    'pos_x', zeros(1, degree+1), ...
    'pos_y', zeros(1, degree+1), ...
    'pos_z', zeros(1, degree+1), ...
    'vel',   zeros(1, degree+1), ...
    't', 0, ...
    'arc_length', 0,...
    'distance', 0, ...
    'polynomial_degree', degree ...
    );

end