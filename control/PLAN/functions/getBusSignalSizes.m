function busSignalSizes = getBusSignalSizes( bus )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2021 Alexander Kuzolap
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

    % Get the field names of the bus to obtain the size of the bus and the
    % signal dimension
    busFields = fieldnames( bus );

    % Get the number of fields
    nFields = length( busFields );

    % Initialize the signal sizes with 1 first
    busSignalSizes = ones( nFields, 1 );

    % Every bus signal could have a dimension, e.g. rates pqr or position xyz as
    % muxed signal. If the signal dimension is bigger than one, increase the max
    % size
    for i = 1:nFields
        busSignal = bus.( busFields{ i } );
        busSignalSize = length( busSignal );
        if busSignalSize > 1
            busSignalSizes(i) = busSignalSize;
        end
    end
end
