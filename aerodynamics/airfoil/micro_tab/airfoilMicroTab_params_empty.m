
% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% weights vector for lift coefficient parameters
micro_tab.net.wcl = [ zeros(1,3), zeros(1,2*1), zeros(1,2*3) ];
% number of neurons in hidden layer
micro_tab.net.ncl = 1;
% number of neural network outputs
micro_tab.net.ocl = 3;


% weights vector for drag coefficient parameters
micro_tab.net.wcd = [ zeros(1,3), zeros(1,2*1), zeros(1,2*3) ];
% number of neurons in hidden layer
micro_tab.net.ncd = 1;
% number of neural network outputs
micro_tab.net.ocd = 3;
% weights vector for drag coefficient parameters


% weights vector for pitching moment coefficient parameters
micro_tab.net.wcm  = [ zeros(1,3), zeros(1,2*1), zeros(1,2*3) ];
% number of neurons in hidden layer
micro_tab.net.ncm = 1;
% number of neural network outputs
micro_tab.net.ocm = 3;

% time constant, in s
micro_tab.T = 1;
