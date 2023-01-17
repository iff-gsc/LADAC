function [Theta_deg, C_T, lambda_i] = ...
bemThetaFromDesiredThrustCoefficentAndInflow(lambda_i, C_T, mu, mu_z, ...
sigma, C_l_alpha)


% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Fabian Guecker
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

lambda      = lambda_i + mu_z;

%% Calculation of Theta from Thrust-Coefficent CT 
% [2] Padfield
% CT for untwisted blade according to [2], p.181 better approximation of
% forward flight

Theta_rad = 3*((2 * C_T  / (sigma * C_l_alpha)) + lambda/2) / (1 + 1.5 *mu^2);


%% Check if mean alpha < 15 deg, else throw an warning
alpha       = -( 3 / 2 * lambda - Theta_rad);

% if (abs(alpha) > 25 / 180 * pi)
%     error('Results may be inaccurate: alpha > 15 deg, possible stall conditions ...')
% end

Theta_deg = Theta_rad * 180 / pi;

end
