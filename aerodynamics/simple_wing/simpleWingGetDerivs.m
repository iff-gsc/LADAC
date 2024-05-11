function derivs = simpleWingGetDerivs(wing,varargin)

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
p       = Coeff_dp * V_0 / (wing.geometry.b/2);
q       = Coeff_dq * V_0 / wing.geometry.c;
r       = Coeff_dr * V_0 / (wing.geometry.b/2);
eta     = Coeff_deta;


derivs = table( alpha, beta, p, q, r, eta, ...
    'RowNames', {'C_X','C_Y','C_Z','C_l','C_m','C_n'} );

end