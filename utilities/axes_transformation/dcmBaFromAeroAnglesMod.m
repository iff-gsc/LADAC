% dcmBaFromAeroAnglesMod computes a modified direction cosine matrix (DCM) from 
% modified aerodynamic angles alpha_M and beta_M
%   The DCM (or rotation matrix) is used to transform a vector given in one
%   frame to another frame by multiplication. The DCM M_ba_M transforms a 
%   vector given in modified aerodynamic frame (a) into body-fixed 
%   frame (b).
% 
% Literature:
% [1]   Stevens, B. L. et al. (2016): Aircraft Control and Simulation.
%       Dynamics, Control Design, and Autonomous Systems. 3rd ed. Wiley.
% [2]   Beyer, Y. (2016): Flugmechanische Modellierung von Multicopter-
%       Systemen in MATLAB/Simulink. Studienarbeit, TU Braunschweig,
%       unpublished.
% 
% Inputs:
%   alpha_M         the scalar modified angle of attack or a vector of n 
%                   modified angles of attack, rad
%   beta_M          the scalar modified sideslip angle or a vector of n 
%                   modified sideslip angles, alpha and beta must be the 
%                   same size, rad
% 
% Outputs:
%   M_ba            direction cosine matrix (with 3x3xn elements) from
%                   aerodynamic frame (a) to body-fixed frame (b), n is the
%                   length of the alpha and beta vector
% 
% See also: aeroAnglesMod
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

function M_ba = dcmBaFromAeroAnglesMod( alpha_M, beta_M ) %#codegen

% detect length of input
len_alpha = length(alpha_M);
% initialize output
M_ba = zeros(3,3,len_alpha);

% for all modified angles of attack (and sideslip angles)
for i = 1:len_alpha
    
	% compute sine and cosine of alpha_M and beta_m (multiple use)
    sin_al = sin(alpha_M(i));
    cos_al = cos(alpha_M(i));
    sin_be = sin(beta_M(i));
    cos_be = cos(beta_M(i));

    % Compute the DCM NOT according to [1, page 77] (note that M_ba = 
    % M_ab'). The order of rotations is inverted. 
    % According to the ISO 1151 (or LN9300), the rotations from aerodynamic
    % frame (a) to body-fixed frame (b) is -beta about z_a, then alpha
    % about y_b. The inverted rotation is alpha_M about y_a, then -beta
    % about z_b [2, page 72-73].
    M_ba(:,:,i) = [cos_al*cos_be, -sin_be, -sin_al*cos_be;
        cos_al*sin_be, cos_be, -sin_al*sin_be;
        sin_al, 0, cos_al ];    

end

end