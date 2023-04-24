function [aic_b,aic_t] = wingGetAicMach( wing, wake, Ma, beta_deg )

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

if strcmp(wing.config.airfoil_method,'analytic')
    fcl = airfoilAnalytic0515Ma( wing.airfoil.analytic.wcl, Ma );
    cla = rad2deg(fcl(2,:));
elseif strcmp(wing.config.airfoil_method,'simple')
    cla = wing.airfoil.simple.c_L_alpha;
else
    error('Airfoil method not supported.')
end

aic_b = indVel2Aic( ind_vel_b, u_n );
aic_t = indVel2Aic( ind_vel_t, u_n );


aic = aic_b;
for i=1:n_trail
    aic = aic + aic_t(:,wing.n_panel*(i-1)+1:wing.n_panel*i);
end

a = aicMachCorrection( aic, Ma, cla, wing.geometry.ctrl_pt.c, wing.params.b );

aic_b = a*aic_b;
aic_t = a*aic_t;

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

function a = aicMachCorrection( AIC, Ma, cla, c_i, b )

beta_Ma = 1./sqrtReal(1-Ma.^2);
beta = diag(beta_Ma);
n_panel = length(c_i);
cla2d = diag(cla);
cla2pi = diag(2*pi*ones(1,n_panel));
c = diag(c_i);

% influence matrix Mach number correction
a = (beta*(-AIC*cla2d*cla2pi*c+2*b*(cla2pi-cla2d))-2*b*(cla2pi-beta*cla2d))/(-AIC*beta*cla2d*cla2pi*c);

end