function omega_gb_b = omegaFromEuler( eulerAngles_dt, ...
    eulerAngles ) %#codegen
% omegaFromEuler computes the first time derivate of the
%   angular velcocity.
% 
% Literature:
%   [1]	Holzapfel, F. (2004). Nichtlineare adaptive Regelung eines 
%       unbemannten Fluggeraetes (Doctoral dissertation, Technische 
%       Universitaet Muenchen).
% 
% Inputs:
%   eulerAngles_dt      vector (3x1) of the first time derivates of the
%                       Euler angles, in rad/s
%   eulerAngles         vector (3x1) of the Euler angles, in rad
% 
% Outputs:
%   omega_gb_b          vector (3x1) of the angular velocity of the body
%                       relative to the earth, in rad/s
% 
% See also: omegaFromEuler2
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

    Phi         = eulerAngles(1);
    Theta       = eulerAngles(2);
    
    sin_Phi     = sin( Phi );
    cos_Phi     = cos( Phi );
    
    sin_Theta   = sin( Theta );
    cos_Theta   = cos( Theta );
        
    M1 = [ 1,            0,         -sin_Theta; ...
           0,      cos_Phi,  sin_Phi*cos_Theta; ...
           0,     -sin_Phi,  cos_Phi*cos_Theta];

    % compute the first time derivative of the Euler angles according to
    % [1, page 131]
    
    omega_gb_b     = M1 * eulerAngles_dt;

end