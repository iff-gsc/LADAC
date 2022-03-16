
% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

%% set atmosphere parameters
% set altitude
h_vec = 0:10:10000;

%% compute turbulence scale length for all parameters
len_h = length(h_vec);
L_u = zeros( len_h, 1 );
L_v = zeros( len_h, 1 );
L_w = zeros( len_h, 1 );
for i_h = 1:len_h
    h = h_vec(i_h);
    [L_u(i_h),L_v(i_h),L_w(i_h)] = turbulenceScaleLength( h );
end

%% plot results
plot( L_u, h_vec )
hold on
plot( L_v, h_vec, '--' )
plot( L_w, h_vec, '-.' )
ylabel('Altitude, in m')
xlabel('Turbulence Scale Length, in m')
legend('L_u','L_v','L_w')