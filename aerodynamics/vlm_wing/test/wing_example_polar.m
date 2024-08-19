% Compute 3D polars for the Arkbird wing for multiple airspeeds (Reynolds
% numbers)

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

wing = wingCreate('wing_params_Arkbird_simple',50);
% note that the "map" option is no longer available for airfoils

V_vec = 5:2:25;

Legend = [];
for i = 1:length(V_vec)
    
    wing.state.body.V = V_vec(i);
    
    polar3D = wingGet3dPolar( wing, deg2rad([-10:1:10]) );
    
    plot( polar3D.C_D, polar3D.C_L );
    Legend{i} = strcat('V = ', num2str(V_vec(i)), 'm/s' );
    hold on
    
end

grid on

legend(Legend)
xlabel('C_D')
ylabel('C_L')