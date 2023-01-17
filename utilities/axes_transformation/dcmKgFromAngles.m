function M_kg = dcmKgFromAngles(gamma,chi)
%#codegen
% DCMkg computes the direction cosine matrix (DCM) for coordinate
% transformation from earth frame (k) frame to flight-path frame (b).
%   The DCM (or rotation matrix) is used to transform a vector given in one
%   frame to another frame by multiplication. 
% 
% Literature:
%   [1] Holzapfel, F. (2004). Nichtlineare adaptive Regelung eines 
%       unbemannten Fluggeraetes (Doctoral dissertation, Technische 
%       Universitaet Muenchen).
%   [1] Brockhaus, R. et al. (2011): Flugregelung. 3rd ed. Springer-Verlag.
% 
% Inputs:
%   gamma       scalar flight-path angle / angle of climb, in rad
%   chi         scalar flight-path azimuth, in rad
% 
% Outputs:
%   M_kg            direction cosine matrix (with 3x3 elements) from
%                   flight-path frame (k) to body-fixed frame (b).
% 
% See also: dcmBkFromAngles
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

    % compute M_kg according to [1, page 233] or [2, page 58]
    M_1g = [cos(chi), sin(chi), 0;...
        -sin(chi), cos(chi), 0;...
        0, 0, 1];
    M_k1 = [cos(gamma), 0, -sin(gamma);...
        0, 1, 0;...
        sin(gamma), 0, cos(gamma)];
    
    M_kg = M_k1 * M_1g;

end