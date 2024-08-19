% This script computes unsteady lift and pitch moment coefficient responses
% due to harmonic pitch oscillation according to [1].
% The resulting figures can be compared with [1], Fig. 5 and Fig. 6.
% One reason of differences is due to a differently chosen lift curve
% slope. This lift curve slope in [1] is based on experimental data (page
% 840), however, these data are not presented in the paper.
% 
% Literature:
%   [1] Leishman, J. G., and Nguyen, K. Q. (1990). State-space 
%       representation of unsteady airfoil behavior. AIAA journal, 28(5),
%       836-844.
% 

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% Vector of to be evaluated Mach numbers
Ma_vec = [ 0.3, 0.5, 0.7, 0.8 ];

% The results do not depend on velocity V and chord c because the reduced
% frequency is used.
V = 1;
c = 1;

% Lift curve slope not given in the paper, that is why the following eq. is
% applied
C_L_alpha_vec = 2*pi./sqrt(1-Ma_vec.^2);

% Aerodynamic center given in Fig. 6
x_ac_vec = [ 0.23, 0.22, 0.23, 0.25 ];

% Init reduced frequency 
k = logspace( -4,log10(0.3),100 );

% init plot handle and figures
h = [];
figure(1)
figure(2)

% compute data for each Mach number
for i = 1:length(Ma_vec)
    
    % compute parameters
    Ma = Ma_vec(i);
    C_L_alpha = interp1( Ma_vec, C_L_alpha_vec, Ma );
    x_ac = interp1( Ma_vec, x_ac_vec, Ma );
    
    % compute state-space representation according to [1]
    [A,B,C,D] = unstAirfoilAero( V, Ma, c, C_L_alpha, x_ac );
    sys_unst = ss( A, B, C, D );
    
    % compute state-space representation for q -> [ alpha; q ]
    % (where q is the dimensionless pitch rate according to [1],
    % Nomenclature)
    A = 0;
    B = V/c;
    C = [ 1; 0 ];
    D = [ 0; 1 ];
    sys_aq = ss( A, B, C, D );
    
    % compute state-space representation for q -> [ C_N; C_m ]
    sys_c = sys_unst * sys_aq;
    
    % compute state-space representation for q -> alpha
    A = 0;
    B = V/c;
    C = 1;
    D = 0;
    sys_a = ss( A, B, C, D );
    
    % compute angular frequency
    % (https://en.wikipedia.org/wiki/Reduced_frequency)
    omega = k * 2*V/c;
    
    % frequency responses
    G_c         = freqresp( sys_c, omega, 'rad/s' );
    G_a         = freqresp( sys_a, omega, 'rad/s' );
    G_N_unst    = freqresp( sys_unst, omega, 'rad/s' );
    
    % compute magnitude of frequency responses normalized by alpha
    abs_G_N_pitch = abs( squeeze(G_c(1,:,:)) ) ./ abs( squeeze(G_a(1,:,:)) );
    abs_G_m_pitch = abs( squeeze(G_c(2,:,:)) ) ./ abs( squeeze(G_a(1,:,:)) );
    % compute phase of frequency response with respect to alpha
    ang_G_N_pitch = unwrap( angle( squeeze(G_c(1,:,:)) ) - angle( squeeze(G_a(1,:,:)) ) );
    ang_G_m_pitch = unwrap( angle( squeeze(G_c(2,:,:)) ) - angle( squeeze(G_a(1,:,:)) ) );
    ang_G_m_pitch(ang_G_m_pitch>pi) = ang_G_m_pitch(ang_G_m_pitch>pi) - 2*pi;
    
    
    % plot data for one Mach number
    figure(1)
    subplot(2,1,1)
    h(i) = plot( k, abs_G_N_pitch );
    xlabel('k')
    ylabel('C_{N,max} / rad (1/rad)')
    hold on
    
    subplot(2,1,2)
    plot( k, ang_G_N_pitch * 180/pi )
    xlabel('k')
    ylabel('\angle C_N (deg)')
    hold on
    
    figure(2)
    subplot(2,1,1)
    plot( k, abs_G_m_pitch )
    xlabel('k')
    ylabel('C_{m,max} / rad (1/rad)')
    hold on
    
    subplot(2,1,2)
    plot( k, ang_G_m_pitch * 180/pi )
    xlabel('k')
    ylabel('\angle C_m (deg)')
    hold on
    
    
end

% plot legend (hard coded)
legend([h],'0.3','0.5','0.7','0.8')