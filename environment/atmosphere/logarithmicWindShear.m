function [ V_w ] = logarithmicWindShear( h, V_w_ref, h_ref, h_0)
% logarithmicWindShear computes the logarithmic wind shear profile.
%   Thus, the wind speed is obtained as a function of the height
%   above surface h = ?z:
% 
% Inputs:
% 	h               altitude (scalar), in m
%
%   V_w_ref         reference wind velocity at href (scalar), in m/s
%
% 	h_ref            reference height (scalar), in m
%
% 	h_0              surface roughness (scalar), in m
% 
% Outputs:
%   V_w             wind velocity (scalar), in m/s
% 
% Literature:
%   [1] G. Sachs. Minimum shear wind strength required for dynamic soaring 
%    of albatrosses. In: Ibis, 147(1):1–10, 2005.
% 
% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2023 Fabian Gücker
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************
% Prevent division by zero
h_0 = max(abs(h_0), eps);

% If reference height is too small, return zero
if(h_ref <= eps)
    h_ref = eps;
end

% If height is below surface roughness, return zero
h(h < h_0) = h_0;

% Calculate wind velocity according to Eq. 7 in [1].
V_w = V_w_ref * log(h ./ h_0) ./ log(h_ref ./ h_0);

end