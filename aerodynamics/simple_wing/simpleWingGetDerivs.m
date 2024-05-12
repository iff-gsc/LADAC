function [derivs,more] = simpleWingGetDerivs(wing,varargin)
% simpleWingGetDerivs get aerodynamic derivatives of simple wing struct
% 
% Syntax:
%   derivs = simpleWingGetDerivs( wing )
%   derivs = simpleWingGetDerivs( wing, Name, Value )
%   [derivs,more] = simpleWingGetDerivs( wing, Name, Value )
% 
% Inputs:
%   wing                Simple wing struct, see simpleWingLoadParams.m
%   Name                Name (string) followed by a value (see below)
%       - 'V_Ab'        Airspeed vector (3x1 array) in body frame, in m/s
%       - 'Omega_Ab'    Angular velocity vector (3x1 array) in body frame,
%                       in rad/s
%       - 'rho'         Air density (scalar), in kg/m^3
%       - 'V_Ab_prop'   Propeller induced velocity (3x2 array) in body
%                       frame, in m/s
%       - 'eta'         Flap deflection vector (1xN array for N flaps), in
%                       rad
%       - 'incidence'   Wing incidence angle (scalar), in rad
%       - 'pos_cg'      Position of simple wing origin measured from center
%                       of gravity in body frame (3x1 array), in m
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
%                       - Coeff_0   Force and moment coefficients (6x1
%                                   array) at specified operating point,
%                                   non-dimensional
% 
% See also:
%   simpleWingCreate, simpleWingLoadParams, simpleWingRun

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2024 Yannic Beyer
%   Copyright (C) 2024 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

V_Ab_0          = [20;0;0];
Omega_0         = zeros(3,1);
rho             = 1.225;
V_Ab_prop       = zeros(3,2);
num_actuators	= length(wing.flap.y_cp_wing);
eta_0           = zeros(1,num_actuators);
incidence       = 0;
pos_cg_b        = zeros(3,1);

p = inputParser;

addParameter(p,'V_Ab',V_Ab_0,@(x) all(size(x)==[3,1]));
addParameter(p,'Omega_Ab',Omega_0,@(x) all(size(x)==[3,1]));
addParameter(p,'rho',rho,@(x) all(size(x)==1));
addParameter(p,'V_Ab_prop',V_Ab_prop,@(x) all(size(x)==[3,2]));
addParameter(p,'eta',eta_0,@(x) length(x)==num_actuators);
addParameter(p,'incidence',incidence,@(x) all(size(x)==1));
addParameter(p,'pos_cg',pos_cg_b,@(x) all(size(x)==[3,1]));

parse(p,varargin{:});

V_Ab_0      = p.Results.V_Ab;
Omega_0     = p.Results.Omega_Ab;
rho         = p.Results.rho;
V_Ab_prop   = p.Results.V_Ab_prop;
eta_0       = p.Results.eta;
incidence   = p.Results.incidence;
pos_cg_b    = p.Results.pos_cg;

[alpha_0,beta_0]    = aeroAngles(V_Ab_0);
V_0                 = norm(V_Ab_0,2);
V_Aa                = [V_0;0;0];


du = 1e-3;

out_0 = simpleWingRun(wing,V_Ab_0,Omega_0,rho,V_Ab_prop,eta_0,incidence,pos_cg_b);

Coeff_0 = [ out_0.dynBody.R_Ab; out_0.dynBody.Q_Ab./[wing.geometry.b;wing.geometry.c;wing.geometry.b] ] ...
                / (wing.geometry.S*mean(out_0.axes.q_P));

alpha_du    = alpha_0 + du;
M_ba_du     = dcmBaFromAeroAngles(alpha_du,beta_0);
V_Ab        = M_ba_du*V_Aa;
out_du      = simpleWingRun(wing,V_Ab,Omega_0,rho,V_Ab_prop,eta_0,incidence,pos_cg_b);
Coeff       = [ out_du.dynBody.R_Ab; out_du.dynBody.Q_Ab./[wing.geometry.b;wing.geometry.c;wing.geometry.b] ] ...
                / (wing.geometry.S*mean(out_du.axes.q_P));
Coeff_dalpha = (Coeff-Coeff_0)/du;

beta_du     = beta_0 + du;
M_ba_du     = dcmBaFromAeroAngles(alpha_0,beta_du);
V_Ab        = M_ba_du*V_Aa;
out_du      = simpleWingRun(wing,V_Ab,Omega_0,rho,V_Ab_prop,eta_0,incidence,pos_cg_b);
Coeff       = [ out_du.dynBody.R_Ab; out_du.dynBody.Q_Ab./[wing.geometry.b;wing.geometry.c;wing.geometry.b] ] ...
                / (wing.geometry.S*mean(out_du.axes.q_P));
Coeff_dbeta = (Coeff-Coeff_0)/du;

Omega_p_du  = Omega_0 + [du;0;0];
out_du      = simpleWingRun(wing,V_Ab_0,Omega_p_du,rho,V_Ab_prop,eta_0,incidence,pos_cg_b);
Coeff       = [ out_du.dynBody.R_Ab; out_du.dynBody.Q_Ab./[wing.geometry.b;wing.geometry.c;wing.geometry.b] ] ...
                / (wing.geometry.S*mean(out_du.axes.q_P));
Coeff_dp    = (Coeff-Coeff_0)/du;

Omega_q_du  = Omega_0 + [0;du;0];
out_du      = simpleWingRun(wing,V_Ab_0,Omega_q_du,rho,V_Ab_prop,eta_0,incidence,pos_cg_b);
Coeff       = [ out_du.dynBody.R_Ab; out_du.dynBody.Q_Ab./[wing.geometry.b;wing.geometry.c;wing.geometry.b] ] ...
                / (wing.geometry.S*mean(out_du.axes.q_P));
Coeff_dq    = (Coeff-Coeff_0)/du;

Omega_r_du  = Omega_0 + [0;0;du];
out_du      = simpleWingRun(wing,V_Ab_0,Omega_r_du,rho,V_Ab_prop,eta_0,incidence,pos_cg_b);
Coeff       = [ out_du.dynBody.R_Ab; out_du.dynBody.Q_Ab./[wing.geometry.b;wing.geometry.c;wing.geometry.b] ] ...
                / (wing.geometry.S*mean(out_du.axes.q_P));
Coeff_dr    = (Coeff-Coeff_0)/du;

Coeff_deta = zeros(6,num_actuators);
for i = 1:num_actuators
    eta_du = eta_0;
    eta_du(i) = eta_du(i) + du;
    out_du	= simpleWingRun(wing,V_Ab_0,Omega_0,rho,V_Ab_prop,eta_du,incidence,pos_cg_b);
    Coeff = [ out_du.dynBody.R_Ab; out_du.dynBody.Q_Ab./[wing.geometry.b;wing.geometry.c;wing.geometry.b] ] ...
                / (wing.geometry.S*mean(out_du.axes.q_P));
    Coeff_deta(:,i) = (Coeff-Coeff_0)/du;
end


alpha   = Coeff_dalpha;
beta    = Coeff_dbeta;
P       = Coeff_dp * V_0 / (wing.geometry.b/2);
Q       = Coeff_dq * V_0 / wing.geometry.c;
R       = Coeff_dr * V_0 / (wing.geometry.b/2);
eta     = Coeff_deta;


derivs = table( alpha, beta, P, Q, R, eta, ...
    'RowNames', {'C_X','C_Y','C_Z','C_l','C_m','C_n'} );

more.Coeff_0 = Coeff_0;

end