% dcm2Euler computes the Euler angles (Phi) from the direction cosine matrix
% (DCM)
%   The Euler angles are used to represent the attitude of the body
%   relative to the earth. They can be computed from the DCM.
% 
% Literature:
% [1]   Stevens, B. L. et al. (2016): Aircraft Control and Simulation.
%       Dynamics, Control Design, and Autonomous Systems. 3rd ed. Wiley.
% 
% Inputs:
%   M_bg            direction cosine matrix (with 3x3 elements) from NED
%                   frame (g) to body frame (b).
% 
% Outputs:
%   EulerAngles     vector with 3 elements containing the Euler angles Phi,
%                   Theta and Psi, rad
% 
% See also: euler2Dcm

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

function EulerAngles  = dcm2Euler( M_bg ) %#codegen

% compute the Euler angles according to [1, page 12]
Phi = atan2( M_bg(2,3), M_bg(3,3) );
Theta = -asinReal( M_bg(1,3) );
Psi = atan2( M_bg(1,2), M_bg(1,1) );

% build the vector of Euler angles
EulerAngles = [ Phi; Theta; Psi ];

end