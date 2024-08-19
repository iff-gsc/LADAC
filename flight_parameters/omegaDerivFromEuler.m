function omega_gb_b_dt = omegaFromEuler2( eulerAngles_dt2, ...
    eulerAngles_dt, eulerAngles ) %#codegen
% omegaFromEuler computes the first time derivate of the
%   angular velcocity.
% 
% Literature:
%   [1]	Holzapfel, F. (2004). Nichtlineare adaptive Regelung eines 
%       unbemannten Fluggeraetes (Doctoral dissertation, Technische 
%       Universitaet Muenchen).
% 
% Inputs:
%   eulerAngles_dt2     vector (3x1) of second time derivates of the
%                       Euler angles, in rad/s
%   eulerAngles_dt      vector (3x1) of the first time derivates of the
%                       Euler angles, in rad/s
%   eulerAngles         vector (3x1) of the Euler angles, in rad
% 
% Outputs:
%   omega_gb_b_dt       vector (3x1) of the first time derivatives of the
%                       angular velocity of the body relative to the earth
%                       represented in earth frame, in rad/s^2
% 
% See also: omegaFromEuler
% 

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

    Phi         = eulerAngles(1);
    Theta       = eulerAngles(2);
    
    Phi_dt      = eulerAngles_dt(1);
    Theta_dt    = eulerAngles_dt(2);
    
    sin_Phi     = sin( Phi );
    cos_Phi     = cos( Phi );
    
    sin_Theta   = sin( Theta );
    cos_Theta   = cos( Theta );
        
    M1 = [ 1,            0,         -sin_Theta; ...
           0,      cos_Phi,  cos_Theta*sin_Phi; ...
           0,     -sin_Phi,  cos_Theta*cos_Phi];
    
    M2 = [ 0,            0,                  0; ...
           0,     -sin_Phi,  cos_Theta*cos_Phi; ...
           0,     -cos_Phi, -cos_Theta*sin_Phi];
    
    M3 = [ 0,            0,         -cos_Theta; ...
           0,            0, -sin_Theta*sin_Phi; ...
           0,            0, -sin_Theta*cos_Phi];
    
    % compute the first time derivative of the Euler angles according to
    % [1, page 132]
    
    omega_gb_b_dt     = (M1 * eulerAngles_dt2) + ...
                        (M2 * eulerAngles_dt)*Phi_dt + ...
                        (M3 * eulerAngles_dt)*Theta_dt;

end