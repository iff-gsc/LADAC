
% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

%% set atmosphere parameters
% set severity exponent
s_exp_vec = [ -0.6990 -1 -2 -3 -4 -5 -6 ];
% set altitude
h_vec = 0:100:40000;
% set mean wind speed 6m above the ground
V_wind_6m = 20;

%% compute RMS turbulence intensity for all parameters
len_h = length(h_vec);
len_s_exp = length(s_exp_vec);
sigma_u = zeros( len_h, len_s_exp );
for i_h = 1:len_h
    for i_exp = 1:len_s_exp
        exp = s_exp_vec(i_exp);
        h = h_vec(i_h);
        [sigma_u(i_h,i_exp),~,~] = turbulenceIntensityRMS( h, exp, V_wind_6m );
    end
end

%% plot results
plot( sigma_u, h_vec )
ylabel('Altitude, in m')
xlabel('RMS Turbulence Velocity, in m/s')