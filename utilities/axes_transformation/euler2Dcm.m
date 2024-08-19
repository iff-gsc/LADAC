% euler2Dcm computes the direction cosine matrix (DCM) from Euler angles(Phi)
%   The DCM (or rotation matrix) is used to transform a vector given in one
%   frame to another frame by multiplication. The DCM M_bg of the functions
%   transforms a vector given in north-east-down (NED) frame (g) to 
%   body-fixed frame (b).
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
%   M_bg            direction cosine matrix (with 3x3 elements) from NED
%                   frame (g) to body frame (b).
% 
% See also: dcm2Euler
% 

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

function M_bg = euler2Dcm(EulerAngles) %#codegen

% compute sine and cosine of each Euler angle (multiple use)
sin_Phi = sin(EulerAngles(1));
cos_Phi = cos(EulerAngles(1));
sin_Theta = sin(EulerAngles(2));
cos_Theta = cos(EulerAngles(2));
sin_Psi = sin(EulerAngles(3));
cos_Psi = cos(EulerAngles(3));

% build the DCM according to [1, page 12]
M_bg = [ cos_Psi*cos_Theta, sin_Psi*cos_Theta, -sin_Theta; ...
         cos_Psi*sin_Theta*sin_Phi-sin_Psi*cos_Phi, ...
         sin_Psi*sin_Theta*sin_Phi+cos_Psi*cos_Phi, cos_Theta*sin_Phi; ...
         cos_Psi*sin_Theta*cos_Phi+sin_Psi*sin_Phi, ...
         sin_Psi*sin_Theta*cos_Phi-cos_Psi*sin_Phi, cos_Theta*cos_Phi];
 
end
