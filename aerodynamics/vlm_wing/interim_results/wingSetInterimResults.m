function interim_results = wingSetInterimResults( wing )
% Description of wingSetInterimResults

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

v_inf = repmat([-1;0;0],1,wing.n_panel);
spanwise_vector = wingGetDimLessSpanwiseLengthVector(wing.geometry.vortex);
interim_results.sweep = pi/2 - acosReal( abs( dot( -spanwise_vector, -v_inf, 1 ) ) ./ ( vecnorm(-v_inf,2,1) .* vecnorm(spanwise_vector,2,1) ) );

spanwise_vector_yz = spanwise_vector;
spanwise_vector_yz(1,:) = 0;
spanwise_vector_yz_average = sum( spanwise_vector_yz, 2 );
spanwise_vector_yz_average = spanwise_vector_yz_average / norm( spanwise_vector_yz_average, 2 );
dot_normal_lateral = dot(spanwise_vector_yz_average,[0;1;0]);
interim_results.wing_x_rotation = mean(acos(dot_normal_lateral).*sign(dot_normal_lateral));
interim_results.M_rot_x = euler2Dcm([interim_results.wing_x_rotation;0;0])';

interim_results.beta_infl = deg2rad(3);
interim_results.dimless_induced_vel_beta = wingGetDimlessIndVel( ...
    repmat(interim_results.M_rot_x*dcmBaFromAeroAngles(0,interim_results.beta_infl)*[-1;0;0],1,wing.n_panel), ...
    wing.state.geometry );
interim_results.dimless_induced_vel = wingGetDimlessIndVel( ...
    repmat(interim_results.M_rot_x*dcmBaFromAeroAngles(0,0)*[-1;0;0],1,wing.n_panel), ...
    wing.state.geometry );
interim_results.Delta_dimless_induced_vel_pos = ...
	interim_results.dimless_induced_vel_beta - interim_results.dimless_induced_vel;
interim_results.dimless_induced_vel_beta = wingGetDimlessIndVel( ...
    repmat(interim_results.M_rot_x*dcmBaFromAeroAngles(0,-interim_results.beta_infl)*[-1;0;0],1,wing.n_panel), ...
    wing.state.geometry );
interim_results.Delta_dimless_induced_vel_neg = ...
	interim_results.dimless_induced_vel_beta - interim_results.dimless_induced_vel;

interim_results.u_n = wingGetNormalVectorFromGeometry( wing.geometry );

end