function [ F_sub, M_sub, P_sub ] = fuselageBall( V_sub, durchmesser, rho)

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Fabian Guecker
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

C_d = 0.45;

area = pi * durchmesser^2 / 4;

F = rho/2 * C_d * area *  norm(V_sub);
F_sub = (V_sub ) * (-F);

M_sub = [0 0 0]';

P_sub = norm(V_sub) * F;

end

