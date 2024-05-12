function [derivs,more] = wingGetDerivs( wing, varargin )
% wingGetDerivs get aerodynamic derivatives of VLM wing struct
% 
% Syntax:
%   derivs = wingGetDerivs( wing )
%   derivs = wingGetDerivs( wing, Name, Value )
%   [derivs,more] = wingGetDerivs( wing, Name, Value )
% 
% Inputs:
%   wing                Simple wing struct (see simpleWingLoadParams)
%   Name                Name (string) followed by a value (see below)
%       -'MomentRefOpt' Moment reference point option with expected
%                       values: 'CoG' (center of gravity; default), 'NP'
%                       (neutral point), 'WingOrigin' (origin of wing),
%                       'Custom' (can be specified via the input
%                       'MomentRefCustom')
%       -'MomentRefCustom' Specification of custom moment reference point
%                       (3x1 array) (only used if input 'MomentRefOpt' is
%                       set to 'Custom'), in m
%       - 'CG'          Center of gravity (3x1 array), default: zeros(3,1),
%                       in m
%       - 'V'           Airspeed vector (scalar), in m/s
%       - 'alpha'       Angle of attack (scalar), in rad
%       - 'beta'        Sideslip angle (scalar), in rad
% 
% Outputs:
%   derivs              Aerodynamic derivatives (table), the rows are
%                       derived w.r.t. the columns, where the rows are the
%                       force and moment coefficients (non-dimensional) and
%                       the columns are:
%                       - alpha     Angle of attack, in rad
%                       - beta      Sideslip angle, in rad
%                       - P         Normalized pitch rate P=p*=p(b/2)/V, 
%                                   non-dimensional
%                       - Q         Normalized roll rate Q=q*=qc/V,
%                                   non-dimensional
%                       - R         Normalized yaw rate R=r*=r(b/2)/V,
%                                   non-dimensional
%                       - eta       Flap deflections, in rad
%   more                More information (struct) with fields:
%                       - x_NP      Longitudinal neutral point position
%                                   w.r.t. moment reference point (scalar),
%                                   in m
%                       - Coeff_0   Force and moment coefficients (6x1
%                                   array) at specified operating point,
%                                   non-dimensional
% 
% See also:
%   wingCreate, wingSetState

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2024 Yannic Beyer
%   Copyright (C) 2024 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

moment_ref_default = {'CoG'};
moment_ref_expected = {'CoG','NP','WingOrigin','Custom'};
moment_ref_custom_default = zeros(3,1);
cog_default = zeros(3,1);
alpha_0     = 0;
beta_0      = 0;
V_0         = 20;

p = inputParser;

addParameter(p,'MomentRefOpt',moment_ref_default,@(x) any(validatestring(x,moment_ref_expected)));
addParameter(p,'MomentRefCustom',moment_ref_custom_default,@(x) all(size(x)==[3,1]));
addParameter(p,'CG',cog_default,@(x) all(size(x)==[3,1]));
addParameter(p,'V',V_0,@(x) length(x)==1);
addParameter(p,'alpha',alpha_0,@(x) length(x)==1);
addParameter(p,'beta',beta_0,@(x) length(x)==1);

parse(p,varargin{:});

moment_ref_opt      = p.Results.MomentRefOpt;
moment_ref_custom	= p.Results.MomentRefCustom;
cog                 = p.Results.CG;
V_0                 = p.Results.V;
alpha_0             = p.Results.alpha;
beta_0              = p.Results.beta;

%%

wing.config.is_unsteady = 0;
wing.geometry.origin(:) = 0;
du = 1e-3;


Omega_0     = zeros(3,1);
num_actuators       = length(wing.state.actuators.pos);
actuators_pos_0     = zeros(1,num_actuators);
actuators_rate_0    = zeros(1,num_actuators);


wing_0 = wingSetState( wing, alpha_0, beta_0, V_0, Omega_0, ...
        actuators_pos_0, actuators_rate_0, cog );

Coeff_0 = [wing_0.state.aero.coeff_glob.C_XYZ_b;wing_0.state.aero.coeff_glob.C_lmn_b];
Coeff_0 = coeffb2a(Coeff_0,alpha_0,beta_0);

alpha_du    = alpha_0 + du;
wing_du     = wingSetState( wing, alpha_du, beta_0, V_0, Omega_0, ...
                actuators_pos_0, actuators_rate_0, cog );
Coeff       = [wing_du.state.aero.coeff_glob.C_XYZ_b;wing_du.state.aero.coeff_glob.C_lmn_b];
Coeff       = coeffb2a(Coeff,alpha_du,beta_0);
Coeff_dalpha = (Coeff-Coeff_0) / du;

beta_du     = beta_0 + du;
wing_du     = wingSetState( wing, alpha_0, beta_du, V_0, Omega_0, ...
                actuators_pos_0, actuators_rate_0, cog );
Coeff       = [wing_du.state.aero.coeff_glob.C_XYZ_b;wing_du.state.aero.coeff_glob.C_lmn_b];
Coeff       = coeffb2a(Coeff,alpha_0,beta_du);
Coeff_dbeta = (Coeff-Coeff_0) / du;

Omega_p_du  = Omega_0 + [du;0;0];
wing_du     = wingSetState( wing, alpha_0, beta_0, V_0, Omega_p_du, ...
                actuators_pos_0, actuators_rate_0, cog );
Coeff       = [wing_du.state.aero.coeff_glob.C_XYZ_b;wing_du.state.aero.coeff_glob.C_lmn_b];
Coeff       = coeffb2a(Coeff,alpha_0,beta_0);
Coeff_dp    = (Coeff-Coeff_0) / du;

Omega_q_du  = Omega_0 + [0;du;0];
wing_du     = wingSetState( wing, alpha_0, beta_0, V_0, Omega_q_du, ...
                actuators_pos_0, actuators_rate_0, cog );
Coeff       = [wing_du.state.aero.coeff_glob.C_XYZ_b;wing_du.state.aero.coeff_glob.C_lmn_b];
Coeff       = coeffb2a(Coeff,alpha_0,beta_0);
Coeff_dq    = (Coeff-Coeff_0) / du;

Omega_r_du  = Omega_0 + [0;0;du];
wing_du     = wingSetState( wing, alpha_0, beta_0, V_0, Omega_r_du, ...
                actuators_pos_0, actuators_rate_0, cog );
Coeff       = [wing_du.state.aero.coeff_glob.C_XYZ_b;wing_du.state.aero.coeff_glob.C_lmn_b];
Coeff       = coeffb2a(Coeff,alpha_0,beta_0);
Coeff_dr    = (Coeff-Coeff_0) / du;

Coeff_du = zeros(6,num_actuators);
for i = 1:num_actuators
    u_du = actuators_pos_0;
    u_du(i) = u_du(i) + rad2deg(du);
    wing_du = wingSetState( wing, alpha_0, beta_0, V_0, Omega_0, ...
        u_du, actuators_rate_0, cog );
    Coeff           = [wing_du.state.aero.coeff_glob.C_XYZ_b;wing_du.state.aero.coeff_glob.C_lmn_b];
    Coeff           = coeffb2a(Coeff,alpha_0,beta_0);
    Coeff_du(:,i)   = (Coeff-Coeff_0) / du;
end


c = wing.params.S/wing.params.b;
x_NP = -Coeff_dalpha(5)/Coeff_dalpha(3) * c;

if strcmp(moment_ref_opt,'NP')
    moment_ref = [x_NP;0;0];
elseif strcmp(moment_ref_opt,'CoG')
    moment_ref = cog;
elseif strcmp(moment_ref_opt,'WingOrigin')
    moment_ref = [0;0;0];
elseif strcmp(moment_ref_opt,'Custom')
    moment_ref = moment_ref_custom;
end

x_NP = x_NP - moment_ref(1);

% Reference conversion from nose center new reference
dist_moment = moment_ref/c;
dist_force = moment_ref/c;

Coeff_dalpha(4:6) = Coeff_dalpha(4:6) - cross(dist_force,Coeff_dalpha(1:3));
Coeff_dbeta(4:6) = Coeff_dbeta(4:6) - cross(dist_force,Coeff_dbeta(1:3));

% Coeff_dq(5) = Coeff_dq(5) + (moment_ref(1))/c * Coeff_dq(3);
Coeff_dq(4:6) = Coeff_dq(4:6) - cross(dist_moment,Coeff_dq(1:3));
Coeff_dr(4:6) = Coeff_dr(4:6) - cross(dist_moment,Coeff_dr(1:3));

for i = 1:num_actuators
    Coeff_du(4:6,i) = Coeff_du(4:6,i) - cross(dist_force,Coeff_du(1:3,i));
end

alpha   = Coeff_dalpha;
beta    = Coeff_dbeta;
P       = Coeff_dp * V_0 / (wing.params.b/2);
Q       = Coeff_dq * V_0 / c;
R       = Coeff_dr * V_0 / (wing.params.b/2);
eta     = Coeff_du;


derivs = table( alpha, beta, P, Q, R, eta, ...
    'RowNames', {'C_X','C_Y','C_Z','C_l','C_m','C_n'} );

more.x_NP = x_NP;
more.Coeff_0 = Coeff_0;


    function Coeff_a = coeffb2a(Coeff_b,alpha,beta)
        M_ba = dcmBaFromAeroAngles(alpha,beta);
        Coeff_a = zeros(6,1);
        Coeff_a(1:3) = M_ba'*Coeff_b(1:3);
        Coeff_a(4:6) = M_ba'*Coeff_b(4:6);
    end

end
