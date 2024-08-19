%  ** default environment parameters **

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

%% environment parameters and initialize environment bus
% altitude of the ground
envir.alt_ground = alt;
% parameters from ISA atmosphere
envir.atmosphere = isAtmosphere( envir.alt_ground );
% wind velocity and acceleration
envir.wind.V_Wg = zeros(3,1);
envir.wind.V_Wg_dt = zeros(3,1);
% earth acceleration, in m/s^2
envir.g = 9.81;