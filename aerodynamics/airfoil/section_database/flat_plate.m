% This script computes a profile_aero struct.
% The computed profile aerodynamics are very simple and based on equations
% for a flat plate with a flap.

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

Ma_vec = 0:0.2:0.9;
alpha_vec = -10:2:10;
act1_vec = -20:4:20;

c_Lalpha0 = 2*pi;
lambda = 0.2;

c_D0 = 0.01;

d_alpha_d_delta = 2/pi*(sqrt(lambda*(1-lambda)) + asin(sqrt(lambda)) );

len_Ma = length(Ma_vec);
len_al = length(alpha_vec);
len_de = length(act1_vec);

c_L_map = zeros(len_al,len_de,len_Ma);
c_D_map = zeros(len_al,len_de,len_Ma);
c_m_map = zeros(len_al,len_de,len_Ma);

for i_Ma = 1:len_Ma
    Ma = Ma_vec(i_Ma);
    for i_al = 1:len_al
        alpha = deg2rad(alpha_vec(i_al));
        for i_de = 1:len_de
            delta = deg2rad(act1_vec(i_de));
            
            beta = sqrt(abs(1-Ma.^2));
            
            c_Lalpha = c_Lalpha0 ./ beta;
            
            c_L = c_Lalpha * ( alpha + d_alpha_d_delta * delta );
            c_D = c_D0 ./ beta;
            c_m = 0;
            
            c_L_map(i_al,i_de,i_Ma) = c_L;
            c_D_map(i_al,i_de,i_Ma) = c_D;
            c_m_map(i_al,i_de,i_Ma) = c_m;
            
        end
    end
end

flat_plate_flap_02.data.c_L = c_L_map;
flat_plate_flap_02.data.c_D = c_D_map;
flat_plate_flap_02.data.c_m = c_m_map;
flat_plate_flap_02.grid.val.x1 = alpha_vec;
flat_plate_flap_02.grid.name.x1 = 'alpha';
flat_plate_flap_02.grid.val.x2 = act1_vec;
flat_plate_flap_02.grid.name.x2 = 'actuator_1';
flat_plate_flap_02.grid.val.x3 = Ma_vec;
flat_plate_flap_02.grid.name.x3 = 'Mach';
