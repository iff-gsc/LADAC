function [T,B,N,kappa,tau] = trajSectionGetFrenetSerretWithGravity ...
    (traj_section, vel, g, varargin)
% trajSectionGetFrenetSerret returns Frenet-Serret apparatus
% of a trajectory section.
%
% Inputs:
%   traj_section   	trajectory section struct, see trajectoryCreate         
%
%   t               dimensionless time paramter (scalar), [0-1]
%
%   vel             tangential velocity (scalar) in m/s
%
%   g               gravitational acceleration is approximately 9.81 m/s^2
%
% Outputs:
%   T               unit vector tangent to the curve, pointing in the
%                   direction of motion.
%                   (3x1 vector), dimensionless
%
%   B               binormal unit vector, the cross product of T and N.
%                   (3x1 vector), dimensionless
%
%   N               normal unit vector, the derivative of T with respect
%                   to the arclength parameter of the curve, divided by its
%                   length.
%                   (3x1 vector), dimensionless
%
%   kappa           curvature
%                   scalar, dimensionless
%
%   tau             torsion
%                   scalar, dimensionless
%
% Syntax:
%   [T,B,H,kappa,tau] = trajSectionGetFrenetSerret(traj_section,vel,G0)
%   [T,B,H,kappa,tau] = trajSectionGetFrenetSerret(traj_section,vel,G0,t)  
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

[T,B,N,kappa,tau] = trajSectionGetFrenetSerretWithAcceleration ...
    (traj_section, vel, [0; 0; -g], t);

end