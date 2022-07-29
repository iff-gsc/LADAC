% dcmBaFromAeroAngles compute direction cosine matrix (DCM) from aerodynamic
% angles alpha and beta
%   The DCM (or rotation matrix) is used to transform a vector given in one
%   frame to another frame by multiplication. The DCM M_ba transforms a 
%   vector given in aerodynamic frame (a) into body-fixed frame (b).
% 
% Literature:
% [1]   Stevens, B. L. et al. (2016): Aircraft Control and Simulation.
%       Dynamics, Control Design, and Autonomous Systems. 3rd ed. Wiley.
% 
% Inputs:
%   alpha           the scalar angle of attack or a vector of n angles of
%                   attack, rad
%   beta            the scalar sideslip angle or a vector of n sideslip
%                   angles, alpha and beta must be the same size, rad
% 
% Outputs:
%   M_ba            direction cosine matrix (with 3x3xn elements) from
%                   aerodynamic frame (a) to body-fixed frame (b), n is the
%                   length of the alpha and beta vector
% 
% See also: aeroAngles
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

function M_ba = dcmBaFromAeroAngles( alpha, beta ) %#codegen

% detect length of input
len_alpha = length(alpha);
% initialize output
M_ba = zeros(3,3,len_alpha);

% for all angles of attack (and sideslip angles)
for i = 1:len_alpha
    
    % compute sine and cosine of alpha and beta (multiple use)
    sin_al = sin(alpha(i));
    cos_al = cos(alpha(i));
    sin_be = sin(beta(i));
    cos_be = cos(beta(i));
    
    % compute the DCM according to [1, page 77] (note that M_ba = M_ab')
    M_ba(:,:,i) = [cos_al*cos_be, -cos_al*sin_be, -sin_al; ...
        sin_be, cos_be, 0; ...
        sin_al*cos_be, -sin_al*sin_be, cos_al ]; 
    
end

end