% Example of a simple INDI rate controller for aircraft/quadcopters

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Fabian Guecker
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

open('INDI_Rate_Controller_Example')

% Reference system natural frequency (PT2)
omega_reference = 10;

% actuator natural frequency (PT2)
omega_actuator = 80;

% Sensor filter natural frequency (PT2)
omega_filter = 100;

% SISO System Dynamics
ss_model.A = 1;
ss_model.B = 5;
ss_model.C = 1;
ss_model.D = 0;

% Control effectiveness
INDI_cntrl.B_inv = 1.0 / ss_model.B;

% Desired System Dynamics
omega_pole = 15;

%% Simple Pole Place, without feedback of highest derivative

% [K_p] = ndiFeedbackGainPlace( -omega_pole*ones(1,1), 0);
% 
% K_p = [K_p, 0]

%% Extended Pole Place, with feedback of highest derivative 

%  Consideration of the filter and Actuator Dynamic
Th = 2/omega_actuator + 2/omega_filter;

[ K_p, A_p, B_p] = ndiFeedbackGainPlace( -omega_pole*ones(2,1) , Th)

ndiPlotClosedLoopErrorDynamics( A_p, B_p, K_p )


