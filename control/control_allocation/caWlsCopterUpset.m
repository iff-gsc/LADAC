function Delta_diag_W_v = caWlsCopterUpset(n_g_des,n_g,ca)

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************
%
% It is probably not the best solution, to set W_v(4) = 0 in case of proj<0
% Could be overhauled (and further investigated) in the future!

Delta_diag_W_v = ca.W_v;
Delta_diag_W_v(:) = 0;

w_aT_max = ca.W_v(4);

w_aT_min = min(ca.W_v(1:2));

proj = dot(n_g,n_g_des);
if proj<0
    w_aT = zeros(1,1,superiorfloat(ca.W_v));
else
    w_aT = w_aT_min + (w_aT_max-w_aT_min)*proj^2;
end

Delta_diag_W_v(4) = w_aT - w_aT_max;

end
