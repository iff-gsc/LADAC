function Delta_diag_W_v = caWlsCopterUpset(n_g_des,n_g,ca)

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

Delta_diag_W_v = diag(ca.W_v);
Delta_diag_W_v(:) = 0;

w_aT_idx = zeros(1,1,superiorfloat(ca.W_v));
w_aT_idx(:) = 4;

w_aT_max = ca.W_v(w_aT_idx,w_aT_idx);
w_aT_min = zeros(1,1,superiorfloat(ca.W_v));
w_aT_min(:) = 10;
w_aT = zeros(1,1,superiorfloat(ca.W_v));

proj = dot(n_g,n_g_des);
if proj<0
    w_aT(:) = 0;
else
    w_aT(:) = w_aT_min + (w_aT_max-w_aT_min)*proj^2;
end
w_aT(:) = w_aT_min + (w_aT_max-w_aT_min);
Delta_diag_W_v(w_aT_idx) = w_aT - ca.W_v(w_aT_idx,w_aT_idx);

end