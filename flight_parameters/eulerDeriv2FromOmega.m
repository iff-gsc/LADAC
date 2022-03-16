function eulerAngles_dt2 = eulerDeriv2FromOmega( omega_gb_b_dt, ...
    omega_gb_b, eulerAngles_dt, eulerAngles ) %#codegen
% eulerAngles_dt2 computes the second time derivate of the Euler angles.
%   Attention: This function is not defined for Theta = +-90 deg (gimbal
%   lock).
% 
% Literature:
%   [1]	Holzapfel, F. (2004). Nichtlineare adaptive Regelung eines 
%       unbemannten Fluggeraetes (Doctoral dissertation, Technische 
%       Universitaet Muenchen).
% 
% Inputs:
%   omega_gb_b_dt       vector (3x1) of the first time derivatives of the
%                       angular velocity of the body relative to the earth
%                       represented in earth frame, in rad/s^2
%   omega_gb_b          vector (3x1) of the angular velocity of the body
%                       relative to the earth, in rad/s
%   eulerAngles_dt      vector (3x1) of the first time derivates of the
%                       Euler angles, in rad/s
%   eulerAngles         vector (3x1) of the Euler angles, in rad
% 
% Outputs:
%   eulerAngles_dt2     vector (3x1) of second time derivatives of the
%                       Euler angles, in rad/s^2
% 
% See also: eulerDerivFromOmega
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

    Theta_max_abs = 0.99*pi/2;
    Phi         = eulerAngles(1);
    Theta       = min( Theta_max_abs, max( -Theta_max_abs, eulerAngles(2) ) );
    Phi_dt      = eulerAngles_dt(1);
    Theta_dt    = eulerAngles_dt(2);
    sin_Phi     = sin( Phi );
    cos_Phi     = cos( Phi );
    cos_Theta   = cos( Theta );
    tan_Theta   = tan( Theta );
    cos_2_Theta = cos_Theta^2;

    M1 = [ 1, sin_Phi*tan_Theta, cos_Phi*tan_Theta; ...
        0, cos_Phi, -sin_Phi; ...
        0, sin_Phi/cos_Theta, cos_Phi/cos_Theta ];
    M2 = [ 0, cos_Phi*tan_Theta, -sin_Phi*tan_Theta; ...
        0, -sin_Phi, -cos_Phi; ...
        0, cos_Phi/cos_Theta, -sin_Phi/cos_Theta ];
    M3 = [ 0, sin_Phi/cos_2_Theta, cos_Phi/cos_2_Theta; ...
        0, 0, 0; ...
        0, sin_Phi*tan_Theta/cos_Theta, cos_Phi*tan_Theta/cos_Theta ];
    
    % compute second time derivative of the Euler angles according to
    % [1, page 133]
    eulerAngles_dt2     = M1 * omega_gb_b_dt ...
                        + Phi_dt * M2 * omega_gb_b ...
                        + Theta_dt * M3 * omega_gb_b;

end