function [first_deriv, sec_deriv, third_deriv] = trajSectionGetDerivatives ...
    (traj_section, varargin)
% trajSectionGetDerivatives returns the first and second derivatives
% of a trajectory section.
%
% Inputs:
%   traj_section   	trajectory section struct, see trajectoryCreate         
%
%   t               dimensionless time paramter 
%                   (scalar), [0-1]
%
% Outputs:
%   first_deriv     first derivative [dxp; dyp; dzp] in local geodetic
%                   system (3x1 vector), dimensionless
%
%   sec_deriv       second derivative [ddxp; ddyp; ddzp] in local geodetic
%                   system (3x1 vector), dimensionless
%
%   third_deriv     second derivative [dddxp; dddyp; dddzp] in local
%                   geodetic system (3x1 vector), dimensionless
%
% Syntax:
%   [acc] = trajSectionGetDerivatives(traj_section)
%   [acc] = trajSectionGetDerivatives(traj_section,t)
%
% See also: trajSectionInit, trajSectionGetAcc, 
%   trajSectionGetLoadFactor
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
    t(:) = varargin{1};
end

% Calculate first derivative of path
dx = polyder(traj_section.pos_x);

% Derivative of y position
dy = polyder(traj_section.pos_y);

% Derivative of z position
dz = polyder(traj_section.pos_z);

first_deriv = [polyVal(dx, t); polyVal(dy, t); polyVal(dz, t)];

% Calculate second derivative of path
ddx = polyder(dx);
ddy = polyder(dy);
ddz = polyder(dz);
sec_deriv = [polyVal(ddx, t); polyVal(ddy, t); polyVal(ddz, t)];

% Calculate third derivative of path
dddx = polyder(ddx);
dddy = polyder(ddy);
dddz = polyder(ddz);
third_deriv = [polyVal(dddx, t); polyVal(dddy, t); polyVal(dddz, t)];

end