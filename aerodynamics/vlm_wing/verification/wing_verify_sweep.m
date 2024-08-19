%% Verifying the nonlinear vortex step method with fundamental solutions:
% 
% Verify the lift curve slope for different sweep and aspect ratio.
% 
% The result should be compared with [1], page 70.
%
% Literature:
%   [1] Schlichting, H.; Truckenbrodt, E.; "Aerodynamik des FLugzeugs -
%       Zweiter Band: Aerodynamik des Tragfluegels (Teil II), des Rumpfes,
%       der Fluegel-Rumpf-Anordnung und der Leitwerke", 3. Auflage,
%       Springer-Verlag, Berlin, Heidelberg, 2001
%

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

%% set wing parameters

% set two vectors for sweep and aspect ratio variation
sweep_vec = deg2rad(-50:10:50);
AR_vec = flip([2,4,6,8,1000]);
AR_vec = ([2,4,6,8,1000]);


% number of panels
n_panel = 40;
n_panel_x = 1;

%% define current wing state

% define current rigid body state
alpha = deg2rad(2.1);
beta = 0;
V = 1;
h = 0;
omega = [0;0;0;];

% center of gravity
xyz_cg = [0;0;0];

% define actuator states
actuators_pos = [0,0];
actuators_rate = [0,0];

%% Start compuation

len_s = length(sweep_vec);
len_A = length(AR_vec);

C_L_alpha = zeros(len_A,len_s);
C_L_alphaVLM = C_L_alpha;

num_total_loops = len_s*len_A;
Legend = [];
% load wing parameters
for i = 1:len_A
    
    for j = 1:len_s
        
        % create wing
        wing = wingCreate( wing_parametric(AR_vec(i),1,sweep_vec(j),0), n_panel, n_panel_x, 'IVLM', 1 );
        
        % compute aerodynamics
        wing = wingSetState(wing,alpha,beta,V,omega,actuators_pos,actuators_rate,[0;0;0]);
        
        % Dimensionless wing coordinates
        eta_wing = wing.geometry.ctrl_pt.pos(2,:) / (wing.params.b / 2);
        
        % lift curve slope
        M_ba = dcmBaFromAeroAngles( wing.state.body.alpha, wing.state.body.beta )';
        C_XYZ_a = M_ba * wing.state.aero.coeff_glob.C_XYZ_b;
        C_L_alpha(i,j) = -C_XYZ_a(3)/alpha;
        
        num_loops = (i-1)*len_s + j;
        disp( ['progress: ',num2str(num_loops / num_total_loops * 100),'%'] )
        
    end
    
    Legend{i} = ['\Lambda = ',num2str(AR_vec(i))];
    
end

%% plot results
figure
plot(rad2deg(sweep_vec),C_L_alpha)
hold on
[C_L_alpha_max,idx_max] = max(C_L_alpha');
plot(rad2deg(sweep_vec(idx_max)),C_L_alpha_max,'kx')
grid on
xlabel(['sweep angle,',char(176)])
ylabel('lift curve slope, -')
legend([Legend,'maximum'],'location','south')
ylim([0 7])
