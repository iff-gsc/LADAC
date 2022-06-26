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
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% [1], eq. (22)
c_m = zeros(size(c_L));
for i = 1:length(c_L)
    c_m(i) = fcm(1,i)+(fcm(2,i)+fcm(3,i)*abs(1-f(i))+fcm(4,i)*sin(pi*f(i)^(fcm(5,i))))*c_L(i);
end

end