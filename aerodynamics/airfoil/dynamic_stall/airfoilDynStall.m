function [ X_dt, c_L, c_m, c_D, f_s, c_N_alpha_max, tau_v_dt, is_leading_edge_shock ] = airfoilDynStall( X, Ma, C_N_p, c_m_p, alpha_E, fcl, fcd, fcm, V, c, alpha, tau_v ) %#codegen
% airfoilDynStall implements a dynamic stall model based on [1] and [2] for
%   airfoils.
%   It is based on the analytic aerodynamic coefficients functions in the
%   airfoilAnalytic0515 project.
%   This dynamic stall model can run for N flow conditions at once.
% 
% Inputs:
%   X_dt        time derivative of the state for dynamic stall model (3xN
%               array)
%   c_L         lift coefficient (1xN array)
%   c_m         pitching moment coefficient (1xN array)
%   c_D         drag coefficient (1xN array)
%   f_s         effective separation point (1xN array)
%   c_N_alpha_max   maximum lift curve slope (1xN array)
%   tau_v_dt    time derivative of the (1xN array)
%   is_leading_edge_shock   bolean indicating the leading edge shock
%                           condition (1xN array of boleans)
% 
% Outputs:
%   X           concentrated states (3xN array)
%   Ma          Mach number (1xN array)
%   c_N_p       total unsteady lift coefficient under attached flow
%               conditions (see unstAirfoilAeroFast) (1xN array), -
%   c_m_p       total unsteady pitching moment coefficient under attached
%               flow conditions (see unstAirfoilAeroFast) (1xN array), -
%   alpha_E     effective angle of attack for unsteady airfoil under
%               attached flow conditions (see unstAirfoilAeroFast) (1xN
%               array), rad
%   fcl         
%   fcd         
%   fcm         
%   V           airspeed (1xN array), m/s
%   c           chord (1xN array), m
%   alpha       angle of attack (1xN array)
%   tau_v       nondimensional time for leading edge separation (1xN array)
% 
% Literature:
%   [1] https://arc.aiaa.org/doi/pdf/10.2514/6.1989-1319
%   [2] https://backend.orbit.dtu.dk/ws/portalfiles/portal/7711084/ris_r_1354.pdf
% 
% See also: unstAirfoilAeroFast, airfoilAnalytic0515LoadParams
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

%% init variables

% nondimensional time (see [1], eq. 32 and subsequent text) derivative w.r.t. time
tau_v_dt = zeros(size(C_N_p));
% init leading edge shock condition
is_leading_edge_shock = false(size(C_N_p));
is_vortex_accumulating = false(size(C_N_p));
fac = 2*V./c;

%% compute parameters

% [1], tab. 1 (explaination in text)
% T_P depends on Ma (to do)
T_P = 1.7*1.05;
% T_f depends on Ma (to do)
T_f = 3;
% T_v and T_vl relatively independent of Ma and airfoil
T_v = 6;
T_vl = 7.5*0.3;

% [1], eq. (17)
a_99    = -fac./T_P;
% [1], eq. (25)
a_10_10 = -fac./T_f;
% [1], eq. (30)
a_11_11 = -fac./T_v;
% [1], eq. (35)
a_12_12 = -fac./T_f;

% [1], eq. (17)
b_9_1 	= fac./T_P;
% [1], eq. (25)
b_10_2 	= fac./T_f;
% [1], eq. (30)
b_11_3  = fac./T_v;
% [1], eq. (35)
b_12_4  = fac*2./T_f;

%% lift coefficient

[c_N_alpha_max,alpha_0] = airfoilAnalytic0515ClAlphaMax( fcl, Ma(:) );
c_N_alpha_max = rad2deg(c_N_alpha_max);
alpha_0_rad = deg2rad(alpha_0);

% [1], eq. (18) or [2], in the text above eq. (21)
C_N_s = X(1,:)';

% [1], eq. (24) or [2], in the text above eq. (21)
alpha_f = C_N_s ./ c_N_alpha_max;

% [1], in text above eq. (25)
c_L_st_f = airfoilAnalytic0515AlCl( fcl, [ rad2deg(alpha_f+alpha_0_rad), Ma(:) ] );

f_s = airfoilDynStallFst( c_L_st_f, deg2rad(c_N_alpha_max), rad2deg(alpha_f) );

% [1], eq. (26)
f_ss = X(2,:)';

c_N_fs = airfoilDynStallClFs( c_L_st_f, c_N_alpha_max, alpha_0_rad, alpha_E, f_s );

% [1], eq. (27) (last term was added for very high angles of attack)
% [3], eq. (23)
c_N_f = c_N_alpha_max .* ((1+sqrtReal(f_ss))/2).^2 .* (alpha_E(:));% .* cos((alpha_E-alpha_0));
% [2], eq. (22) (probably easier?)
% C_N_f = C_N_alpha_max * (alpha_E-alpha_0) * f_ss ... %.* cos(pi/2/90*(alpha_E-alpha_0) ...
%     + C_N_fs .* (1-f_ss) + pi/fac*alpha_dt;

% Set C_N_C = C_N_f because this yields the static coefficients in case of
% very slow frequencies (C_N_C is explained in the text below [1], eq. (28)).
% However, note that C_N_C is not clearly defined in [1].
c_N_C = c_N_f;

% [1], eq. (31)
c_N_v = X(3,:)';
% [1], eq. (37)
c_N = c_N_f + c_N_v;

% leading edge shock condition
c_L_max = airfoilAnalytic0515ClMax( fcl, Ma(:) );
is_leading_edge_shock( C_N_s>c_L_max | ( tau_v>1e-3 & c_N_v>1e-3 ) ) = 1;
tau_v_dt(is_leading_edge_shock) = fac(is_leading_edge_shock);
is_vortex_accumulating(is_leading_edge_shock & tau_v<T_vl) = 1;

% [3], eq. (27)
K_N = (1+sqrtReal(f_ss)).^2/4;
% [3], eq. (26)
c_v = zeros(size(c_N_C));
c_v(is_vortex_accumulating) = c_N_C(is_vortex_accumulating) .* (1-K_N(is_vortex_accumulating));

%% drag coefficient
% difficult, difficult, difficult, ...

% [2], eq. (24)
alpha_E_0 = alpha_E(:) + alpha_0_rad;
Delta_c_D_ind = sin( alpha - alpha_E_0 ) .* c_N;
% Delta_c_D_ind = c_N * sin(alpha);

% [3], eq. (25)
% c_C_f = 0.95 * c_N_alpha_max .* alpha_E_0.^2 .*sqrtReal(f_ss);

c_L = c_N .* cos(alpha_E);

% parameters for [2], eq. (26)
% c_D_st_E = airfoilAnalytic0515AlCd( fcd, rad2deg(alpha_E_0) );
c_D_st = airfoilAnalytic0515AlCd( fcd, rad2deg(alpha) );

% c_D_0 = airfoilAnalytic0515Cd0( fcd );

% c_L_st_E = airfoilAnalytic0515AlCl( fcl, [rad2deg(alpha_E_0), Ma ] );
% f_st_E = airfoilDynStallFst( c_L_st_E, deg2rad(c_N_alpha_max), rad2deg(alpha_E) );

% [2], eq. (26)
% this has shown to have only a very small impact (probably it is important
% for really strong increase of drag coefficient)
% Delta_c_D_fss = (c_D_st_E-c_D_0) .* ...
%     ( ((1-sqrtReal(f_ss))/2).^2 - ((1-sqrtReal(f_st_E))/2).^2 );

% C_C_f = (c_D_st_E-c_D_0) .*  ( sqrt(f_ss) - sqrt(f_s) );

c_D = unstAirfoilAeroCdNoFlutter( c_D_st, c_N, alpha, c_L_st_f );


%% pitching moment coefficient

% [1], eq. (28)
c_M_f = airfoilAnalyticBlCm( fcm, f_ss, c_L );

CP_v = 0.2 * ( 1 - cos(min(pi,pi*tau_v./(T_vl))) );
c_m_v = -CP_v .* c_N_v;

c_m = c_m_p + c_M_f + c_m_v;


%% update state space model

% to do: u = [ C_N_p(t); f'(t); C_v_dt; f_qs(t) ]
u = [ C_N_p'; f_s'; c_v' ];

X_dt = cat( 1, a_99', a_10_10', a_11_11' ) .* X ...
    + cat( 1, b_9_1', b_10_2', b_11_3' ) .* u;

end