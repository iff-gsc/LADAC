function c_m = airfoilAnalyticBlCm(fcm,f,c_L)
% airfoilAnalyticBlAlCm returns the pitching moment coefficient according to
% the analytic Beddoes-Leishman model [1].
% 
% Inputs:
%   fcm         analytic function parameters array (see outputs of
%               airfoilAnalyticBlAlFit)
%   f           separation point location (1xN array) (see output of
%               airfoilDynStallFst), dimensionless
%   c_L         lift coefficient (1xN array), dimensionless
% 
% Outputs:
%   c_m         pitching moment coefficient (1xN array), dimensionless
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

% [1], eq. (22)
c_m = fcm(1,:) + ( ...
    fcm(2,:) ...
    + fcm(3,:) .* abs(1-f) + ...
    + fcm(4,:) .* sin(pi*f.^(fcm(5,:))) ) .* c_L;

end