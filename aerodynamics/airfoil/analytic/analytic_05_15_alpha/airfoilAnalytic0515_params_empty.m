
% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% weights vector for lift coefficient parameters
airfoil.wcl = [ zeros(1,6), zeros(1,2*1), zeros(1,2*6) ];
% number of neurons in hidden layer
airfoil.ncl = 1;
% number of neural network outputs
airfoil.ocl = 6;


% weights vector for drag coefficient parameters
airfoil.wcd = [ zeros(1,6), zeros(1,2*1), zeros(1,2*6) ];
% number of neurons in hidden layer
airfoil.ncd = 1;
% number of neural network outputs
airfoil.ocd = 6;
% weights vector for drag coefficient parameters


% weights vector for pitching moment coefficient parameters
airfoil.wcm  = [ zeros(1,5), zeros(1,2*1), zeros(1,2*5) ];
% number of neurons in hidden layer
airfoil.ncm = 1;
% number of neural network outputs
airfoil.ocm = 5;
