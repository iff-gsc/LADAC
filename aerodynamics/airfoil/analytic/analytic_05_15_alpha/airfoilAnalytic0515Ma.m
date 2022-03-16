function fMa = airfoilAnalytic0515Ma( weights, Ma, numNeurons, numOutputs )
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
%   numNeurons      total number of neural network neurons
%   numOutputs      number of neural network outputs (equivalent to number
%                   of analytic function ceofficients)
% 
% Outputs:
%   fMa             concentrated coefficients of analytic function for all
%                   Mach numbers (Mx(numOutputs) array)
% 
% See also: airfoilAnalytic0515NeunFit, airfoilAnalytic0515AlCl
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% normalization factors
idx1 = numOutputs;

% end of first layer
idx2 = idx1 + numNeurons*2;

fMa = repmat( weights(1:idx1), length(Ma), 1 );

% forward propagation
inputHiddenLayer = matrixMultiplyNd( reshape( weights(idx1+1:idx2), [], 2 ), ...
    cat( 1, reshape(Ma,1,1,[]), ones(1,1,length(Ma)) ) );

outputHiddenLayer = tanh( inputHiddenLayer );

fMa = fMa .* reshape( ...
    matrixMultiplyNd( reshape( weights(idx2+1:end), [], numNeurons+1 ), ...
    cat( 1, outputHiddenLayer, ones(1,1,size(outputHiddenLayer,3)) ) ), ...
    numOutputs, [] )';

end