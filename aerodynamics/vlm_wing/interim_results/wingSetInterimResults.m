function interim_results = wingSetInterimResults( wing, Ma_inf )
% Description of wingSetInterimResults

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

v_inf = repmat([-1;0;0],1,wing.n_panel);
spanwise_vector = wingGetDimLessSpanwiseLengthVector(wing.geometry.line_25);
interim_results.sweep = pi/2 - acosReal( abs( dot( -spanwise_vector, -v_inf, 1 ) ) ./ ( vecnorm(-v_inf,2,1) .* vecnorm(spanwise_vector,2,1) ) );
spanwise_flap_vector = wingGetDimLessSpanwiseLengthVector(wing.geometry.vortex);
interim_results.sweep_flap = pi/2 - acosReal( abs( dot( -spanwise_flap_vector(:,:,end), -v_inf, 1 ) ) ./ ( vecnorm(-v_inf,2,1) .* vecnorm(spanwise_flap_vector(:,:,end),2,1) ) );
local_incidence = [wing.geometry.ctrl_pt.local_incidence(1,1,1),wing.geometry.ctrl_pt.local_incidence(1,1:end-1,1)+diff(wing.geometry.ctrl_pt.local_incidence(1,:,1))/2,wing.geometry.ctrl_pt.local_incidence(1,end,1)];
pos_50.pos = wing.geometry.line_25.pos + [-cos(local_incidence);zeros(size(local_incidence));sin(local_incidence)].*wing.geometry.line_25.c/4;
pos_50.c = wing.geometry.line_25.c;
spanwise_vector_50 = wingGetDimLessSpanwiseLengthVector(pos_50);
interim_results.sweep50 = pi/2 - acosReal( abs( dot( -spanwise_vector_50, -v_inf, 1 ) ) ./ ( vecnorm(-v_inf,2,1) .* vecnorm(spanwise_vector_50,2,1) ) );
% for the LEISA aircraft a spanwise constant Mach number showed best
% agreement
x_root = interp1(pos_50.pos(2,:)/max(pos_50.pos(2,:)),pos_50.pos(1,:),-0.1,'linear');
y_root = interp1(pos_50.pos(2,:)/max(pos_50.pos(2,:)),pos_50.pos(2,:),-0.1,'linear');
interim_results.sweep50(:) = atan((pos_50.pos(1,1)-x_root)/(pos_50.pos(2,1)-y_root));

Ma = Ma_inf * cos(interim_results.sweep50);

spanwise_vector_yz = spanwise_vector;
spanwise_vector_yz(1,:) = 0;
spanwise_vector_yz_average = sum( spanwise_vector_yz, 2 );
spanwise_vector_yz_average = spanwise_vector_yz_average / norm( spanwise_vector_yz_average, 2 );
dot_normal_lateral = dot(spanwise_vector_yz_average,[0;1;0]);
interim_results.wing_x_rotation = mean(acos(dot_normal_lateral).*sign(dot_normal_lateral));
interim_results.M_rot_x = euler2Dcm([interim_results.wing_x_rotation;0;0])';

n_trail = 1;
interim_results.wake.pos_x      = linspace( 0, 3*wing.params.b, n_trail+1 );
interim_results.wake.dx         = interim_results.wake.pos_x(2)-interim_results.wake.pos_x(1);

interim_results.wake.pos_start  = wing.geometry.vortex.pos(:,:,end);

interim_results.beta_infl = deg2rad(1);

n_panel_x = 1;

interim_results.u_n = wingGetNormalVectorFromGeometry( wing.geometry );


[AIC_b_beta,AIC_t_beta] = wingGetAicMach( wing, interim_results.wake, Ma, interim_results.beta_infl );
[AIC_b,AIC_t] = wingGetAicMach( wing, interim_results.wake, Ma, 0 );

Delta_AIC_b_pos = AIC_b_beta - AIC_b;
Delta_AIC_t_pos = AIC_t_beta - AIC_t;

[AIC_b_beta,AIC_t_beta] = wingGetAicMach( wing, interim_results.wake, Ma, -interim_results.beta_infl );

Delta_AIC_b_neg = AIC_b_beta - AIC_b;
Delta_AIC_t_neg = AIC_t_beta - AIC_t;

interim_results.AIC_b = AIC_b;
interim_results.AIC_t = AIC_t;
interim_results.Delta_AIC_b_pos = Delta_AIC_b_pos;
interim_results.Delta_AIC_t_pos = Delta_AIC_t_pos;
interim_results.Delta_AIC_b_neg = Delta_AIC_b_neg;
interim_results.Delta_AIC_t_neg = Delta_AIC_t_neg;

end