function [T,B,N,kappa,tau] = trajSectionGetFrenetSerret ...
    (traj_section, varargin)
% trajSectionGetFrenetSerret returns Frenet-Serret apparatus
% of a trajectory section.
%
% Inputs:
%   traj_section   	trajectory section struct, see trajectoryCreate         
%
%   t               dimensionless time paramter 
%                   (scalar), [0-1]
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
%   [T,B,H,kappa,tau] = trajSectionGetFrenetSerret(traj_section)
%   [T,B,H,kappa,tau] = trajSectionGetFrenetSerret(traj_section,t)
% 
% Literature:
%   [1] K. Meyberg, P. Vachenauer (2001): Hoehere Mathematik 1
%       (Sechste, korrigierte Auflage.). Springer Berlin Heidelberg.       
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

[dot_r, ddot_r, dddot_r] = trajSectionGetDerivatives ...
    (traj_section, t);

% pre-calculation, often used
cross_dot_r_ddot_r = cross(dot_r, ddot_r);

% unit vector tangent to the curve
T = dot_r / norm(dot_r);

% binormal unit vector
B = cross_dot_r_ddot_r / norm(cross_dot_r_ddot_r);

% normal unit vector
N = cross(B, T);

% curvature
kappa = norm(cross_dot_r_ddot_r) / ( norm(dot_r)^3 );

% torsion
tau = det([dot_r, ddot_r, dddot_r]) / ( norm(cross_dot_r_ddot_r)^2 );

end