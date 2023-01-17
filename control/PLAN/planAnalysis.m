function [ A, B, C, D, K ] = planAnalysis( states_dtPertub, ...
    controlInputsPertub, outputsPertub, pertubDelta, i )
%#codegen
%planAnalysis   computes the linearized state-space representation matrices
%   due to pertubation.
%
% Inputs:
%   states_dtPertub         pertubated time derivatives of the state vector
%   controlInputsPertub     pertubated control inputs vector
%   outputsPertub           pertubated output vector
%   pertubDelta             current pertubation value (scalar)
%   i                       current pertubation index (scalar)
%
% Output:
%   A                       system matrix
%   B                       input matrix
%   C                       output matrix
%   D                       feedthrough matrix
%   K                       control gain matrix
%
% See also: planInput
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2021 Alexander Kuzolap
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

    %% Declare persistent variable
    persistent iMax iStatesMax;
    persistent states_dtDelayed ...
        controlInputsDelayed outputsDelayed;
    
    persistent A_per B_per C_per D_per K_per;
    
    %% Initialization
    % Get the signal sizes
    numStates = length( states_dtPertub );
    numCntrIn = length( controlInputsPertub );
    numOutputs = length( outputsPertub );
        
    A = zeros( numStates );
    B = zeros( numStates, numCntrIn );
    C = zeros( numOutputs, numStates );
    D = zeros( numOutputs, numCntrIn );
    K = zeros( numCntrIn, numStates );
    
    if isempty( iMax ) || isempty( iStatesMax )
        
        iStatesMax = numStates;
        iMax = iStatesMax + numCntrIn;
    end
    

    % Initialize the delayed bus
    if isempty( states_dtDelayed )
        states_dtDelayed = states_dtPertub;
        controlInputsDelayed = controlInputsPertub;
        outputsDelayed = outputsPertub;
    end
    
    if isempty( A_per )
        A_per = A;
    end
    if isempty( B_per )
        B_per = B;
    end
    if isempty( C_per )
        C_per = C;
    end
    if isempty( D_per )
        D_per = D;
    end
    if isempty( K_per )
        K_per = K;
    end

    %% Analysis
    if i == 0
        % If all signals were pertubated, update the delayed bus to current
        % bus input
        states_dtDelayed = states_dtPertub;
        controlInputsDelayed = controlInputsPertub;
        outputsDelayed = outputsPertub;
        
    elseif ( i <= ( iStatesMax  ) ) && not( isequal( i,0 ) )
        %% Pertubation of the states
        % 1) Pertubation states_dt
        % 2) Pertubation outputs
        % 3) Pertubation control inputs
        %% A matrix
        deltaStates_dt = states_dtPertub - states_dtDelayed;
        statesIdx = 1:numStates;
        A_per( statesIdx, i ) = deltaStates_dt / pertubDelta;
        
        %% C matrix
        deltaOutputs = outputsPertub - outputsDelayed;
        outputsIdx = 1:numOutputs;
        C_per( outputsIdx, i ) = deltaOutputs / pertubDelta;
        
        %% K Matrix
        deltaCntrlInputs = controlInputsPertub - controlInputsDelayed;
        K_per( 1:end, i ) = deltaCntrlInputs / pertubDelta;
        
    elseif not( isequal( i,0 ) )
        %% Pertubation of the control input
        % 1) Pertubation outputs
        % 2) Pertubation control inputs
        
        col = i - iStatesMax;
        
        %% D matrix
        deltaOutputs = outputsPertub - outputsDelayed;
        outputsIdx = 1:numOutputs;
        D_per( outputsIdx, col ) = deltaOutputs / pertubDelta;
        
        %% B matrix
        deltaStates_dt = states_dtPertub - states_dtDelayed;        
        statesIdx = 1:numStates;
        B_per( statesIdx, col ) = deltaStates_dt / pertubDelta;
    end
    A = A_per;
    B = B_per;
    C = C_per;
    D = D_per;
    K = K_per;

end