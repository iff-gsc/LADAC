function fMa = airfoilAnalytic0515Ma( weights, Ma )
% airfoilAnalytic0515Ma returns the coefficients for analytic functions in
% the airfoilAnalytic0515 project for different Mach number based on the
% weights of a shallow (one hidden layer) neural network.
%   The computation of the coefficients based on a neural network is
%   usually faster than interpolation of stored data.
% 
% Inputs:
%   weights         weights of the neural network (note that the first
%                   elements are no weights but maximum input values used
%                   for normalization) (1xN array)
%   Ma              vector of Mach numbers (Mx1 or 1xM array)
% 
% Outputs:
%   fMa             concentrated coefficients of analytic function for all
%                   Mach numbers ((numOutputs)xM array)
% 
% See also: airfoilAnalytic0515NeunFit, airfoilAnalytic0515AlCl
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

num_inputs = length(Ma);
bias_in = ones(1,num_inputs);

% forward propagation
inputHiddenLayer = weights.weights1 * [Ma;bias_in];

outputHiddenLayer = tanh( inputHiddenLayer );

fMa = weights.weights2 * [ outputHiddenLayer; bias_in ];

end