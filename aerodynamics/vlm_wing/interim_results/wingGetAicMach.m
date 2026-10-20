function [aic_b,aic_t,aic_trefftz] = wingGetAicMach( wing, wake, Ma, beta_deg )

u_n = wingGetNormalVectorFromGeometry( wing.geometry );
spanwise_vector = wingGetDimLessSpanwiseLengthVector(wing.geometry.line_25);
n_trail = length(wake.pos_x)-1;
n_panel_x = 1;

spanwise_vector_yz = spanwise_vector;
spanwise_vector_yz(1,:) = 0;
spanwise_vector_yz_average = sum( spanwise_vector_yz, 2 );
spanwise_vector_yz_average = spanwise_vector_yz_average / norm( spanwise_vector_yz_average, 2 );
dot_normal_lateral = dot(spanwise_vector_yz_average,[0;1;0]);

wing_x_rotation = mean(acos(dot_normal_lateral).*sign(dot_normal_lateral));
M_rot_x = euler2Dcm([wing_x_rotation;0;0])';

V_inf_local = beta2localInflow( M_rot_x, beta_deg, wing.n_panel, n_panel_x );

[ind_vel_b,ind_vel_t] = wingGetDimlessIndVel( ...
    V_inf_local, wing.state.geometry, wake, Ma, wing.config.is_unsteady );

wing_mod = wing;
wing_mod.state.geometry.ctrl_pt.pos(1,:) = wing_mod.state.geometry.ctrl_pt.pos(1,:) - 0.5*wake.pos_x(2);
wing_mod.state.geometry.ctrl_pt.pos(3,:) = wing.geometry.vortex.pos(3,1:end-1,end) ...
    + diff(wing.geometry.vortex.pos(3,:,end))/2;
[ind_vel_b_trefftz,ind_vel_t_trefftz] = wingGetDimlessIndVel( ...
    V_inf_local, wing_mod.state.geometry, wake, Ma, wing.config.is_unsteady );
ind_vel_trefftz = ind_vel_b_trefftz + ind_vel_t_trefftz;

aic_b = indVel2Aic( ind_vel_b, u_n );
aic_t = indVel2Aic( ind_vel_t, u_n );
aic_trefftz = indVel2Aic( ind_vel_trefftz, u_n );

aic = aic_b;
for i=1:n_trail
    aic = aic + aic_t(:,wing.n_panel*(i-1)+1:wing.n_panel*i);
end

end



function V_inf_local = beta2localInflow( M_rot_x, beta_deg, n_panel, n_panel_x )

beta = deg2rad(beta_deg);

V_inf_1 = M_rot_x * dcmBaFromAeroAngles(0,beta) * [-1;0;0];
V_inf_local = repmat( V_inf_1, 1, n_panel, n_panel_x );

end

function aic = indVel2Aic(ind_vel,u_n)

aic = zeros(size(ind_vel,2),size(ind_vel,3));
% influence coefficients matrix (Katz & Plotkin, Eq. (12.7))
for i = 1:size(ind_vel,3)
    aic(:,i) = dot( ind_vel(:,:,i), u_n, 1 );
end

end
