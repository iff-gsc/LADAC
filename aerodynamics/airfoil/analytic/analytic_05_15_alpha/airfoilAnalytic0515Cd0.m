function [c_D_0,alpha_0] = airfoilAnalytic0515Cd0(fcd)

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

c_D_0 = fcd(:,1);
alpha_0 = fcd(:,3);

end