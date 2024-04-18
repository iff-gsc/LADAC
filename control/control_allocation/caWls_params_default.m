% ** Parameters for wls control allocation (default) **

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% k is the number of control inputs
% m is the number of pseudo control inputs

% minimum control input (used for k control inputs)
param.u_min = 0.1;

% maximum control input (used for k control inputs)
param.u_max = 1;

% desired control input (used for k control inputs)
param.u_d = 0.1;

% weighting mx1 array of pseudo-control (used as mxm matrix)
param.W_v = [10; 10; 0.01; 1];

% weighting kx1 array of the control input vector (used as kxk matrix)
param.W_u = ones(4,1);

% weighting of pseudo-control vs. control input (scalar)
param.gamma = 1000;

% maximum number of iterations (scalar)
param.i_max = 100;
