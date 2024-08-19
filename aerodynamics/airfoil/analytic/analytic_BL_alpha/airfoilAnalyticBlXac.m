function x_ac = airfoilAnalyticBlXac(beta)
% aerodynamic center from parameters of analytic pitching moment
%   coefficient function
% 
% Literature:
%   [1] https://arc.aiaa.org/doi/pdf/10.2514/6.1989-1319
% 
% Inputs:
%   beta(1)    	pitching moment coefficient at zero lift (c_m0)
%   beta(2)   	K0
%   beta(3)    	K1
%   beta(4)   	K2
%   beta(5)     m
% 
% Outputs:
%   x_ac        aerodynamic center measured from the airfoil nose, positive
%               backwards, normalized by the chord length
% 

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************


% [1], below eq. (22)
x_ac = 0.25 - beta(2,:);

end