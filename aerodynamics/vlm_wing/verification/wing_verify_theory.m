%% Verifying the nonlinear vortex step method with fundamental solutions
% 
% Compare aspect ratio, taper ratio and sweep with results from literature.
% 
% Literature:
% [1]   Nickel, K.; Wohlfahrt, W.; "Schwanzlose Flugzeuge - Ihre Auslegung
%       und ihre Eigenschaften"; 1. Auflage, Birkh�user Verlag Basel, 1990
% [2]   Schlichting, H.; Truckenbrodt, E.; "Aerodynamik des FLugzeugs -
%       Zweiter Band: Aerodynamik des Tragfl�gels (Teil II), des Rumpfes,
%       der Fl�gel-Rumpf-Anordnung und der Leitwerke", 3. Auflage,
%       Springer-Verlag, Berlin, Heidelberg, 2001
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

%% define current wing state

% define current rigid body state
alpha = deg2rad(2.1);
beta = deg2rad(0);
V = 1;
omega = [0;0;0;];

% center of gravity
xyz_cg = [0;0;0];

% define actuator states
actuators_pos = [0,0];
actuators_rate = [0,0];


%% aspect ratios
% compare to [2], p.46, fig. 7.23

figure_aspect_ratio = figure;

aspect_ratio = [6,9,12];

Legend = cell( length(aspect_ratio),1 );
for index_wing = 1:length(aspect_ratio)

    % create wing
    wing = wingCreate( wing_parametric(aspect_ratio(index_wing),1,0,0), 50 );

    % set state
    wing = wingSetState( wing, alpha, beta, V, omega, actuators_pos, actuators_rate, xyz_cg );

    % Dimensionless wing coordinates
    eta_wing = wing.geometry.ctrl_pt.pos(2,:) / (wing.params.b / 2);

    k_r = find( eta_wing >= 0 );

    plot(eta_wing(k_r),wing.state.aero.coeff_loc.c_XYZ_b(3,k_r)/wing.state.aero.coeff_glob.C_XYZ_b(3));
    title(["Aspect ratios","(compare to [2], p.46, fig. 7.23)"])
    ylim([0,1.3])
    xlabel('\eta, -')
    ylabel('c_L/c_{L,mean}, -')
    grid on
    
    Legend{index_wing} = strcat('AR = ', num2str(aspect_ratio(index_wing)) );

    hold on
end
legend(Legend)


%% taper ratios
% compare to [2], p.47, fig. 7.25

figure_taper = figure;

taper_ratio = [ 1, 0.5, 0.25, 0 ];

Legend = cell( length(taper_ratio),1 );
for index_wing = 1:length(taper_ratio)

    % create wing
    wing = wingCreate( wing_parametric(6,taper_ratio(index_wing),0,0), 50 );

    % set state
    wing = wingSetState( wing, alpha, beta, V, omega, actuators_pos, actuators_rate, xyz_cg );

    % Dimensionless wing coordinates
    eta_wing = wing.geometry.ctrl_pt.pos(2,:) / (wing.params.b / 2);

    k_r = find( eta_wing >= 0 );

    subplot(2,1,1)
    title(["Taper ratios","(compare to [2], p.47, fig. 7.25)"])
    plot(eta_wing(k_r),wing.state.aero.circulation.gamma(k_r)/alpha);
    ylabel('\gamma/\alpha, -')
    grid on
    hold on
    subplot(2,1,2)
    plot(eta_wing(k_r),wing.state.aero.coeff_loc.c_XYZ_b(3,k_r)/wing.state.aero.coeff_glob.C_XYZ_b(3));
    grid on
    ylim([0,1.6])
    xlabel('\eta, -')
    ylabel('c_L/c_{L,mean}, -')
    Legend{index_wing} = strcat('\tau = ', num2str(taper_ratio(index_wing)) );
    hold on
end
legend(Legend,'Location','southwest')
    
    
%% sweepback angles
% compare to [2], p.71

figure_sweep = figure;

sweep_angle = [ 0, deg2rad(45) ];

Legend = cell( length(sweep_angle),1 );
for index_wing = 1:length(sweep_angle)

    % create wing
    wing = wingCreate( wing_parametric(5,1,sweep_angle(index_wing),0), 50 );

    % set current rigid body state
    wing = wingSetState( wing, alpha, beta, V, omega, actuators_pos, actuators_rate, xyz_cg );

    % Dimensionless wing coordinates
    eta_wing = wing.geometry.ctrl_pt.pos(2,:) / (wing.params.b / 2);

    k_r = find( eta_wing >= 0 );

    subplot(2,1,1)
    title(["Sweep angles","(compare to [2], p.71)"])
    plot(eta_wing(k_r),wing.state.aero.circulation.gamma(k_r)/alpha);
    grid on
    hold on
    ylim([0,inf])
    xlabel('\eta, -')
    ylabel('\gamma/\alpha, -')
    subplot(2,1,2)
    plot(eta_wing(k_r),-wing.state.aero.coeff_loc.c_XYZ_b(3,k_r)/alpha/2/wing.params.AR);
    Legend{index_wing} = strcat('\phi = ', ...
        num2str(rad2deg(wing.params.lambda)), '�' );
    grid on
    ylim([0,inf])
    xlabel('\eta, -')
    ylabel('c_L/\alpha/2/\Lambda, -')
    hold on
end
legend(Legend,'Location','southwest')

    
%% twist angles
% comparison to established results is missing

figure_twist = figure;

twist_angle = deg2rad( [ -10, -5, 0, 5, 10 ] );

Legend = cell( length(twist_angle),1 );
for index_wing = 1:length(twist_angle)

    % create wing
    wing = wingCreate( wing_parametric(6,1,0,twist_angle(index_wing)), 50 );

    % set current rigid body state
    wing = wingSetState( wing, 0, beta, V, omega, actuators_pos, actuators_rate, xyz_cg );

    % Dimensionless wing coordinates
    eta_wing = wing.geometry.ctrl_pt.pos(2,:) / (wing.params.b / 2);

    k_r = find( eta_wing >= 0 );

    plot(eta_wing(k_r),wing.state.aero.circulation.gamma(k_r)/alpha);
    title(["Twist angles","(comparison to established results is missing)"])
    grid on
    hold on
    xlabel('\eta, -')
    ylabel('\gamma/\alpha, -')
    Legend{index_wing} = strcat('\epsilon = ', ...
        num2str(rad2deg(wing.params.epsilon)), '�' );
    grid on
    hold on
end
legend(Legend,'Location','best')


%% elliptic wing
% comparison to established results is missing

figure_elliptic = figure;

Legend = cell( length(1),1 );
for index_wing = 1:length(1)

    % create wing
    wing = wingCreate( wing_parametric(6,1,0,0), 50, 'is_elliptical', true );

    % set current rigid body state
    wing = wingSetState( wing, alpha, beta, V, omega, actuators_pos, actuators_rate, xyz_cg );

    % Dimensionless wing coordinates
    eta_wing = wing.geometry.ctrl_pt.pos(2,:) / (wing.params.b / 2);

    k_r = find( eta_wing >= 0 );
    
    subplot(2,1,1)
    plot(eta_wing(k_r),wing.state.aero.circulation.gamma(k_r)/alpha);
    grid on
    hold on
    ylim([0,inf])
    xlabel('\eta, -')
    ylabel('\gamma/\alpha, -')
    grid on
    hold on
    
    subplot(2,1,2)
    plot(eta_wing(k_r),-wing.state.aero.coeff_loc.c_XYZ_b(3,k_r)/alpha/2/wing.params.AR);
    title(["Elliptic wing"])
    grid on
    hold on
    ylim([0,inf])
    xlabel('\eta, -')
    ylabel('c_L/\alpha/2/\Lambda, -')
    Legend{index_wing} = strcat('\epsilon = ', ...
        num2str(rad2deg(wing.params.epsilon)), '�' );
    grid on
    hold on
end
