% ** Parameters for wls control allocation (default) **

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% k is the number of control inputs
% m is the number of pseudo control inputs

% minimum control input kx1 vector
param.u_min = 0.1*ones(4,1);
% maximum control input kx1 vector
param.u_max = ones(4,1);
% desired control input kx1 vector
param.u_d = 0.1*ones(4,1);

% weighting mxm matrix of pseudo-control
param.W_v = diag([10,10,0.01,1]);
% weighting kxk matrix of the control input vector
param.W_u = eye(4);
% weighting of pseudo-control vs. control input (scalar)
param.gamma = 1000;
% initial working set mx1 vector
param.W = zeros(4,1);
% maximum number of iterations (scalar)
param.i_max = 100;
