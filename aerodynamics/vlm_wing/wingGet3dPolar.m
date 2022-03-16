function polar3D = wingGet3dPolar( wing, alpha )

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

len_al = length(alpha);

C_L = zeros(1,len_al);
C_D = zeros(1,len_al);
C_m = zeros(1,len_al);

for i = 1:length(alpha)

    % set angle of attack
    wing.state.body.alpha = alpha(i);

    %% compute aerodynamic state

    wing = wingSetAeroState( wing, [0;0;0] );

    % save result
    M_ab = dcmBaFromAeroAngles( wing.state.body.alpha, wing.state.body.beta )';
    C_XYZ_a = M_ab * wing.state.aero.coeff_glob.C_XYZ_b;
    C_lmn_a = M_ab * wing.state.aero.coeff_glob.C_lmn_b;
    C_L(i) = - C_XYZ_a(3);
    C_D(i) = - C_XYZ_a(1);
    C_m(i) = C_lmn_a(2);

end

% save result
polar3D.C_L = C_L;
polar3D.C_D = C_D;
polar3D.C_m = C_m;

end