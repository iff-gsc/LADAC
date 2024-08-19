% euler2Quat computes the quaternions from Euler angles.
%   Both Euler angles and quaternions can be used to describe the attitude
%   of a frame with reference to another frame. This function computes the
%   quaternions from given Euler angles according to the ISO 1151 (or
%   LN9300).
% 
% Literature:
% [1]   Stevens, B. L. et al. (2016): Aircraft Control and Simulation.
%       Dynamics, Control Design, and Autonomous Systems. 3rd ed. Wiley.
% 
% Inputs:
%   EulerAngles     vector with 3 elements containing the Euler angles Phi,
%                   Theta and Psi, rad
% 
% Outputs:
%   q_bg            four dimensional quaternion from g frame to b frame
% 
% See also: quat2Dcm
% 

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

function q_bg = euler2Quat(EulerAngles) %#codegen

% compute sine and cosine of Euler angles (multiple use)
sin_Phi_2 = sin(EulerAngles(1)/2);
cos_Phi_2 = cos(EulerAngles(1)/2);
sin_Theta_2 = sin(EulerAngles(2)/2);
cos_Theta_2 = cos(EulerAngles(2)/2);
sin_Psi_2 = sin(EulerAngles(3)/2);
cos_Psi_2 = cos(EulerAngles(3)/2);

% compute quaternion according to [1, page 52]
q_bg = [ cos_Phi_2*cos_Theta_2*cos_Psi_2 + ...
        sin_Phi_2*sin_Theta_2*sin_Psi_2;...
        sin_Phi_2*cos_Theta_2*cos_Psi_2 - ...
        cos_Phi_2*sin_Theta_2*sin_Psi_2;...
        cos_Phi_2*sin_Theta_2*cos_Psi_2 + ...
        sin_Phi_2*cos_Theta_2*sin_Psi_2;...
        cos_Phi_2*cos_Theta_2*sin_Psi_2 - ...
        sin_Phi_2*sin_Theta_2*cos_Psi_2 ];
    
end
