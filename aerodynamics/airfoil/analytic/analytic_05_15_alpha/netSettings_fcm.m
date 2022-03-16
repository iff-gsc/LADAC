% netSettings_default   default network parameter settings file.
%   This file contains the default settings to setup the neural network.
%   Various parameters stored in a struct with the name netPrm. Only parameters 
%   in netPrm are loaded.
%
%   Basic neural network setup parameter:
%       blockID         The blockID is used inside the matlab function to
%                       identify the network. You cannot use the same ID for
%                       different networks
%       numInputs       Define the number of network inputs
%       numOutputs      Define the number of network outputs that the
%                       network have to match
%       numNeuronsInHiddenLayers    Define the number of neurons in hidden
%                                   layers. This is an 1xN array containing
%                                   the number of neurons in each layer
%       inputScalingParameters      TODO
%       outputScalingParameters     TODO
%   Setup Network weights:
%       weights         This parameter can be used to initialize or load
%                       existing weights stored in a file. This parameter
%                       is not required. If this parameter is not given the
%                       random weights are loaded from the default range from 
%                       -.1 to +.1. If weights parameters are used in createNet 
%                       function the settings given in the netPrm struct will be 
%                       will be overwritten and ignored.
%                           'random'    Loads random weighst in the default 
%                                       range, if the parameters weightsRangeMin
%                                       and weightsRangeMax not
%                                       given.
%                           'filename   Loads the weights from the given
%                                       file. TODO file path and file
%                                       specifications.
%       weightsRangeMin,weightsRangeMax This setup the lower and upper bound for
%                                       initial random weights. Both
%                                       parameters should be set. If this
%                                       parameter is not set the default
%                                       range is used. If the 'range'
%                                       Parameter is used in createNet function
%                                       the settings given in the netPrm struct
%                                       will be  will be overwritten and 
%                                       ignored.
%   Intervall parameter:
%       numPatterns         TODO
%       patternInterval     TODO
%       numValidation       TODO
%   Settings for the training:
%       trainMethod         0 : No training
%                           1 : Levenberg Marquardt
%                           2 : Gradient decent
%                           3 : Resilient Backpropagation
%                           4 : Lyapunov asymptotic stable training
%                           5 : Sliding Mode Control with gradient decent
%       offsetTime          Start time of the training after which training is
%                           performed given in seconds.
%       trainingDeadZoneMSE Maximal MSE error that is required to perform 
%                           training
%       learnRate           TODO
%       momentum            Momentum (only for gradient decent)
%       lambda              Sliding Control Parameter parameter lambda from the 
%                           switching function
%       adaptLearnRate      ?? TODO
%                           Set adaptive learning rate:
%                               false : Fix
%                               true  : Adaptive
%	Settings for sample time:
%       sampleTime  Setup the sample time. Use -1 for inheriting the sample time 
%                   from parent block.
%   See also createNet
%
%   Example
%       netPrm.blockID = 1;
%       netPrm.numInputs = 1;
%       netPrm.numOutputs = 1;
%       netPrm.numNeuronsInHiddenLayers = [ 20 15 ];
%       ...

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

%% Setup the network parameter
% Write ID to netPrm
netPrm.blockID = 1;
%netPrm.numNetworks = numNetworks;

% Set the number of input
netPrm.numInputs = 1;
% Set the number of outputs
netPrm.numOutputs = 5;
% Set number of hidden layer
netPrm.numNeuronsInHiddenLayers = [ 4 ];
% Set the activation function for the differnt layer
% 1 : identity
% 2 : sigmoid
% 3 : tanh
% 4 : gaussian
% 41 : narrow gaussian
netPrm.activationFcn = [ 1 3 1 ];
% Define the input scaling function
netPrm.inputScalingParameters = [ -1; 1 ];
% Define the output scaling function
netPrm.outputScalingParameters = zeros( 2, netPrm.numOutputs );
netPrm.outputScalingParameters(1,:) = -1;
netPrm.outputScalingParameters(2,:) = 1;

%% Setup Network weights
% Select method for weight initilization
%   (1) : random
netPrm.weights = 1;
netPrm.weightsRangeMin = -.10;
netPrm.weightsRangeMax = +.10;
%% Intervall parameter
% Intervall for the training
netPrm.numPatterns = 5;
% Recordintervall for the net data
netPrm.patternInterval = 1;
% Number of saved pattern
netPrm.numValidation = 5;

%% Settings for the training
% Select online/stochastic or offline/batch training
%   (0) : batch (only supported for training algorithm 1,2,10
%   (1) : stochastic
netPrm.training.online = 0;
% Select number of patterns for weight update (only utilized for 
% (mini)batch training). If the weights should not be updated, select inf. 
% The weights which are not updated are stored in TDATA.
netPrm.training.miniBatchSize = 14;
% Set the training algorithm
%   (0) : No training
%   (1) : Gradient decent
%   (2) : Gradient decent momentum
%   (3) : Lyapunov asymptotic stable training
%   (10) : Resilient Backpropagation
%   (20) : Levenberg Marquardt
%   (30) : Sliding Mode Control with gradient decent
%   (40) : Adam (Adaptive Moment Estimation)
netPrm.training.trainAlgorithm = 10;
% Start time of the training after which training is performed given in seconds
netPrm.training.offsetTime = 1.;
% Maximal MSE error that is required to perform training
netPrm.training.trainingDeadZoneMSE = 1.E-05;
% (Start-) learning rate
%TODO Check where it is used and why start learning rate
% Note: If a training algorithm using local learning rates is selected,
% an Scell array of the scalar learning rate with the dimension of the
% weights will be generated.
netPrm.training.learnRate = 5.E-02;
% Momentum (only for gradient decent)
netPrm.training.momentum = 0.;
% Sliding Control Parameter parameter lambda from the switching function
netPrm.training.lambda = 10.;
% epsilon (only applicable for Adam, RMSProp, AdaGrad, AdaDelta, ...)
netPrm.training.epsilon = 1.;
% Settings for Resilient Propagation
netPrm.training.increaseFactor = 1.2;
netPrm.training.decreaseFactor = 0.2;
% settings for Adam
netPrm.training.firstMomentDecay = 0.9;
netPrm.training.secondMomentDecay = 0.999;
% Set adaptive learning rate
%   (false) : Fix
%   (true)  : Adaptive
netPrm.training.adaptLearnRate = true;
% set the regularization parameter
netPrm.training.lambdaReg = 1.e-6;
% Set linearization (partial derivative of the neural network output w.r.t.
% the neural network input)
%   (false) : no linearization will be performed
%             (patterns.dOutput_dInput = 0)
%   (true)  : the neural network will be linearized
%             (patterns.dOutput_dInput)
% If a uint8 equals 1 is used, the NET.para.training and therefore NET could
% not be used as a Bus Object.
netPrm.training.performLin = uint8(1);
% Computing of the jacobian matrix
%   (false) : no calculation of the jacobian matrix will be performed
%   TODO          (patterns.dOutput_dInput = 0)
%   (true)  : the jacobian matrix will be calculated
%             (patterns.dOutput_dInput)
% If a uint8 equals 1 is used, the NET.para.training and therefore NET could
% not be used as a Bus Object.
netPrm.training.jacobian = uint8(1);

%% Settings for adaptive learning rate
%TODO Check whether it is needed and used and where it is used
% Change factor of learning rate
%netPrm.changeFactor=10.;
% Maximal learning rate
%netPrm.learnMax=1.E+08;
% Minimal learning rate
%netPrm.learnMin=1.E-10;

%% Settings for inverse propagation
%TODO Check whether it is needed and used and where it is used
% Type of inverse propagation
%netPrm.invPropID = 2;
%   (1) : Krogmann
%   (2) : Using inverse Transferfunctions
% Type of backpropagated value
%netPrm.invPropValue = 1;
%   (1) : Absolute value, similar to the network output
%   (2) : Deviation from the absolute value (small values)

%% Settings for sample time
% Use -1 for inheriting the sample time from parent
netPrm.sampleTime = -1;
% ******************************************************************************