function eulerAngles_dt = eulerDerivFromOmega( omega_gb_b, eulerAngles ) 
%#codegen
% eulerDerivFromOmega computes the first time derivate of the Euler angles.
%   Attention: This function is not defined for Theta = +-90 deg (gimbal
%   lock).
% 
% Literature:
%   [1]	Stevens, B. L. et al. (2016): Aircraft Control and Simulation.
%       Dynamics, Control Design, and Autonomous Systems. 3rd ed. Wiley.
% 
% Inputs:
%   omega_gb_b          vector (3x1) of the angular velocity of the body
%                       relative to the earth, in rad/s
%   eulerAngles         vector (3x1) of the Euler angles, in rad
% 
% Outputs:
%   eulerAngles_dt      vector (3x1) of first time derivatives of the
%                       Euler angles, in rad/s^2
% 
% See also: eulerDeriv2FromOmega
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

    Phi         = eulerAngles(1);
    Theta       = eulerAngles(2);
    sin_Phi     = sin( Phi );
    cos_Phi     = cos( Phi );
    cos_Theta   = cos( Theta );
    tan_Theta   = tan( Theta );

    M = [ 1, sin_Phi*tan_Theta, cos_Phi*tan_Theta; ...
        0, cos_Phi, -sin_Phi; ...
        0, sin_Phi/cos_Theta, cos_Phi/cos_Theta ];
    
    % compute first time derivative of Euler angles according to 
    % [1, page 20]
    eulerAngles_dt = M * omega_gb_b;

end