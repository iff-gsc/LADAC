function M_bk = dcmBkFromAngles(mu_k,alpha_k,beta_k)
%#codegen
% DCMbk computes the direction cosine matrix (DCM) for coordinate
% transformation from flight-path (k) frame to body-fixed frame (b).
%   The DCM (or rotation matrix) is used to transform a vector given in one
%   frame to another frame by multiplication. For the flight-path
%   aerodynamic angles mu_k, alpha_k, beta_k the definition of [1] and [2]
%   is used which is different from [3].
% 
% Literature:
%   [1] Holzapfel, F. (2004). Nichtlineare adaptive Regelung eines 
%       unbemannten Fluggeraetes (Doctoral dissertation, Technische 
%       Universitaet Muenchen).
%   [2]	Beyer, Y., Kuzolap, A., Steen, M., Diekmann, J. H., & Fezans, N. 
%       (2018). Adaptive Nonlinear Flight Control of STOL-Aircraft Based on
%       Incremental Nonlinear Dynamic Inversion. In 2018 Modeling and 
%       Simulation Technologies Conference (AIAA 6.2018-3257).
%   [3] Brockhaus, R. et al. (2011): Flugregelung. 3rd ed. Springer-Verlag.
% 
% Inputs:
%   mu_k        scalar flight-path bank angle, in rad
%   alpha_k     scalar flight-path angle of attack, in rad
%   beta_k      scalar flight-path sideslip angle, in rad
% 
% Outputs:
%   M_bk            direction cosine matrix (with 3x3 elements) from
%                   flight-path frame (k) to body-fixed frame (b).
% 
% See also: dcmKgFromAngles
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

    % To do: perform multiplication for higher efficiency
    % compute M_bk according to [1, page 234] or [2, page 14]
    % ATTENTION: M_bk is NOT computed according to [3, page 59]
    M_1k = [1, 0, 0; ...
        0, cos(mu_k), sin(mu_k); ...
        0, -sin(mu_k), cos(mu_k)];
    M_21 = [cos(beta_k), -sin(beta_k), 0; ...
        sin(beta_k), cos(beta_k), 0; ...
        0, 0, 1];
    M_b2 = [cos(alpha_k), 0, -sin(alpha_k); ...
        0, 1, 0; ...
        sin(alpha_k), 0, cos(alpha_k)];

    M_bk = M_b2 * M_21 * M_1k;

end