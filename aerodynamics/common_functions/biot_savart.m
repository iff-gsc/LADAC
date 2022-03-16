function  [v_induced] = biot_savart (x_p, x_1, x_2, nu, t)
%biot_savart calculates velocity induced by vortex line.
%   The function biot_savart calculates the velocity vector induced at the 
%   point x_p by a vortex line of unit strength from the points x_1 to x_2
%   after the Biot-Savart law.
%   biot_savart, in order to avoid infinities and astronomical quantities 
%   for zero or small distances, resp., assumes a finite vortex core which 
%   is calculated from the kinematic viscosity of the air nu (in m^2/s) and 
%   the vortex age t (in s) according to [2] and [3] (approximation of 
%   Lamb-Oseen vortex). 
%
% Syntax:  [v_induced] = biot_savart (x_p, x_1, x_2, nu, t)
% 
% Literature: 
%   [1] McCormick, Barnes W.: FORTRAN subroutine to nonlin.m program.
%       1999.
%   [2] Heintsch, Thomas: Beitraege zur Modellierung von Wirbelschleppen
%       zur Untersuchung des Flugzeugverhaltens beim Landeanflug.
%       Dissertation, TU Braunschweig, 1994.
%   [3] Leishman, Gordon J.; Bhagwhat, Mahendra J.; Bagai, Ashish:
%       Free-Vortex Filament Method for the Analysis of Helicopter Rotor
%       wakes. J Aircraft, 39(5), 2002.
%
% Inputs:
% 	 x_p                    Control point at which induced velocity is to 
%                           be calculated (3-D vector), in m
%                           (array)
%    x_1                    End of vortex line: left panel boundary 
%                           coordinates (3-D vector), in m
%                           (array)
%    x_2                    Start of vortex line: respective right panel 
%                           coordinates (3-D vector), in m
%                           (array)
%    nu                     Air kinematic viscosity, in m�/s 
%                           (double)
%    t                      Vortex age, in s
%                           (double)
%
% Outputs:
%    v_induced              Velocity induced by vortex line (3-D vector), 
%                           im m/s
%                           (double
%
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% (absolute) tolerance for small values, in m
tol = 1e-6;

%% check input arguments
if (nargin < 3)
    error ('3 input vectors required.')
end
if ~(isequal(size(x_p), [3 1]) && isequal(size(x_1), [3 1]) && ...
        isequal(size(x_2), [3 1]))
    error('The first 3 input variables must be 3-D vectors (i.e. 3x1-arrays).')
end

% set defaults for optional arguments
% kinematic viscosity
if (nargin >= 4)
    % if nu is empty
    if isempty(nu)
        % set viscosity nu from ISA for H=0
        nu = 1.4607186e-5;                     
    end
end

% vortex age
if (nargin == 5)
    % if t is empty
    if isempty(t)
        % set vortex age to 0s
        t = 0;
    end
end

%% calculate vectors
% vector x_1 -> p, in m
a = x_p - x_1; 
% vector p -> x_2, in m
b = x_2 - x_p;
% vector x_1 -> x_2, in m
c = x_2 - x_1;                             

% check whether points coincide
if ((norm(a) < tol) || (norm(b) < tol) || (norm(c) < tol))
    error('None of the 3 points may coincide.')
end

% direction of induced velocity
v = cross(c, a);

% (cosine of) angle between a and c
cos_alpha = a'*c / (norm(a) * norm(c));
% (cosine of) angle between b and c
cos_beta  = b'*c / (norm(b) * norm(c));

% avoid numerical problems when cos alpha is near to unity
if (abs(cos_alpha) > (1 - tol))
    x_3 = x_1 + norm(a) / norm(c) * (x_2 - x_1);
    h = norm(x_3 - x_p);
else
    % distance p -> vortex line
    h = norm(a) * sqrt(1 - cos_alpha^2);
end

%% calculate induced velocity

if ((h < tol) || (norm(v) < tol))
    % check whether p is on (or very close to) vortex line
    v_induced = [0; 0; 0];
%     warning('p must not be on the vortex line itself.')
else
    if (nargin >= 4)
        % calculate viscous vortex core
        % core radius for Lamb-Oseen vortex taking into account fictious 
        % initial vortex age of 40 s after [2]
        r_c = sqrt(4 * 1.25643 * nu * (40 + t));

        % de-singularisation factor after [3, p.4], equation (15)
        K = h^2 / sqrt(r_c^4 + h^4);          
    else
        % calculate potential vortex
        K = 1;
    end
    
    % induced velocity (magnitude of induced velocity)
    v_induced = v / norm(v) / (4 * pi) / h * K * (cos_alpha + cos_beta);
end                                       

return