function [ statesPertub, controlInputsPertub, statesNotPertub, ...
    pertubDelta, i_out ]...
    = planPertubator( statesPertub, controlInputsPertub, statesNotPertub, ...
    statesPertubDelta, controlInputPertubDelta )
%#planPertubator   Pertubates the given signals for the linearization
%   This function ...
%
% Inputs:
%   statesPertub            state vector or bus that shall be pertubated
%	controlInputs           control inputs vector or bus that shall be 
%                           pertubated
%	statesNotPertub         state vector that shall NOT be pertubated
%	statesPertubDelta       state vector or bus pertubation signal
%	controlInputsDelta      control inputs vector of bus pertubation signal
%
% Output:
%	statesPertub            pertubated state vector or bus
%	controlInputs           pertubated control inputs vector or bus
%	statesNotPertub         adhered state vector or bus
%	i_out                   pertubation index (scalar)
%
% See also: planAnalysis

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2021 Alexander Kuzolap
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

    %%
    persistent i iMax iPlntMax iCntrInMax;
    persistent statesPertubSigSizes controlInputsSigSizes;
    persistent statesPertubSigCumsum controlInputsSigCumsum;
    persistent statesPertubDelayed controlInputsDelayed ...
        statesNotPertubDelayed;
    
    pertubDelta = 0;
   
    %% Initialization
    
    % check if inputs are bus
    isBus = ~isa( statesPertub, 'double' );
    
    % Initialize the counter
    if isempty( i )
        % Get the bus signal sizes, because every bus signal could have a 
        % dimension, e.g. rates pqr or position xyz as muxed signal.
        if isBus
            statesPertubSigSizes = getBusSignalSizes( statesPertub );
            controlInputsSigSizes = getBusSignalSizes( controlInputsPertub );
        else
            statesPertubSigSizes = length( statesPertub );
            controlInputsSigSizes = length( controlInputsPertub );
        end
        
        statesPertubSigCumsum = cumsum( statesPertubSigSizes );
        controlInputsSigCumsum = cumsum( controlInputsSigSizes );
        
        iPlntMax = sum( statesPertubSigSizes );
        iCntrInMax = sum( controlInputsSigSizes );
        iMax = iPlntMax + iCntrInMax;
    end
           
    % Initialize the delayed bus
    if isempty( statesPertubDelayed )
        statesPertubDelayed = statesPertub;
        controlInputsDelayed = controlInputsPertub;
        statesNotPertubDelayed = statesNotPertub;
    end

    %% Manipulate the bus and pertubate
    if isempty( i )
        % Initialize the counter
        i = 0;
        
    elseif i == iMax
        % If all signals were pertubated, update the delayed bus to current
        % bus input
        i = 0;
        statesPertubDelayed = statesPertub;
        controlInputsDelayed = controlInputsPertub;
        statesNotPertubDelayed = statesNotPertub;
    else
        % Increment the counter
        i = i + 1;
       
        if isBus
            % Get the field names to access the bus values
            stateFields = fieldnames( statesPertub );
            controlInputFields = fieldnames( controlInputsPertub );
        end
        
        % Copy the delayed buses to the current bus
        statesPertub = statesPertubDelayed;
        controlInputsPertub = controlInputsDelayed;
        statesNotPertub = statesNotPertubDelayed;
                
        % Pertubate the signals plant states, controller states and control
        % input
        if i <= iPlntMax

            if isBus
                % Obtain the current signal that have to be pertubated
                found = ( i - statesPertubSigCumsum ) <= 0;
                sigNum = find( found, 1, 'first' );
            
                for j = 1:length( stateFields )

                    % Find the right signal that have to be pertubated
                    if  isequal(sigNum, j)

                        % Obtain the signal array index
                        sigIdx = i - sum( statesPertubSigSizes( 1:(sigNum(1) - 1) ) );

                        % Obtain the current delta
                        pertubDelta(1,1) = statesPertubDelta.( stateFields{ j } )(sigIdx);

                        % Get the whole signal to be able to write it back
                        plantState = statesPertub.( stateFields{ j } );
                        % Manipulate the element in the signal with the pertubation
                        plantState( sigIdx ) = plantState( sigIdx ) + pertubDelta;
                        % Write the fun back to bus signal
                        statesPertub.( stateFields{ j } ) = plantState;
                    end
                end
                    
            else
                idx = i;
                pertubDelta(1,1) = statesPertubDelta(idx);
                statesPertub(idx) = statesPertub(idx) + pertubDelta; 
            end
            
        else
            
            if isBus
                % Obtain the current signal that have to be pertubated
                found = ( ( i - iPlntMax ) - controlInputsSigCumsum ) <= 0;
                sigNum = find( found, 1, 'first' );

                for j = 1:length( controlInputFields )

                    if isBus
                        if  isequal(sigNum, j)

                            % Obtain the signal array index
                            sigIdx = ( i - iPlntMax ) - sum( controlInputsSigSizes( 1:(sigNum(1) - 1) ) );

                            % Obtain the current delta
                            pertubDelta(1,1) = controlInputPertubDelta.( controlInputFields{ j } )(sigIdx);

                            % Get the whole signal to be able to write it back
                            controlInput = controlInputsPertub.( controlInputFields{ j } );
                            % Manipulate the element in the signal with the pertubation
                            controlInput( sigIdx ) = controlInput( sigIdx ) + pertubDelta;
                            % Write the fun back to bus signal
                            controlInputsPertub.( controlInputFields{ j } ) = controlInput;
                        end
                    end     
                end
            else
                idx = i - iPlntMax;
                pertubDelta(1,1) = controlInputPertubDelta(idx);
                controlInputsPertub(idx) = controlInputsPertub(idx) + pertubDelta;
            end
        end
    end
    
    i_out = uint32(i);
end