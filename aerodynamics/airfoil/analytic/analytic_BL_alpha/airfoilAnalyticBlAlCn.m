function c_N = airfoilAnalyticBlAlCn(fcn,x)
% airfoilAnalyticBlAlCn returns the normal force coefficient according to
% the analytic Beddoes-Leishman model [1].
% 
% Inputs:
%   fcn         analytic function parameters array (see output of
%               airfoilAnalyticBlAlFit)
%   x           angle of attack (1xN array), in deg
% 
% Outputs:
%   c_N         normal force coefficient (1xN array), dimensionless
% 
% Literature:
%   [1] https://arc.aiaa.org/doi/pdf/10.2514/6.1989-1319
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

c_N_alpha = fcn(1);
alpha_0 = fcn(2);
alpha_1 = fcn(3);
S_1 = fcn(4);
S_2 = fcn(5);
eta = fcn(6);

alpha = x(:)';

% [1], eq. (21)
f = zeros(size(alpha));
idx1 = alpha < alpha_1;
f(idx1) = 1 - 0.3 * exp((alpha(idx1)-alpha_1)/S_1);
f(~idx1) = 0.04 + 0.66 * exp((alpha_1-alpha(~idx1))/S_2);

c_N = c_N_alpha * ((1+sqrt(f))/2).^2 .* (alpha-alpha_0); %sin(pi/90*alpha);
c_C = eta * c_N_alpha * (alpha*pi/180).^2 .* sqrt(f) .* cos(pi/180*alpha);
% c_L = sign(c_N).*sqrt(c_N.^2+c_C.^2);
c_L = c_N .* cos(pi/180*alpha) + c_C .* sin(pi/180*alpha);

end