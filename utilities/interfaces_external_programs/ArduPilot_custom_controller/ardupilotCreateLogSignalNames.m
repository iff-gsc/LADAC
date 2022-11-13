function [signal_names_1D,signal_names,num_signals] = ...
    ardupilotCreateLogSignalNames( block_path, num_signals_vec )
% ardupilotCreateLogSignalNames converts ArduPilot Log names to numbers
%   Simulink signals can not be chars or strings. That is why each letter
%   is converted to data type uint8 by this function.
%   This function is usually called by the functions ardupilotInitLogMuxer
%   and ardupilotInitLogMuxerCore. These functions are used for
%   initialization of the Simulink block "log muxer". Have a look at this
%   block for more information.
% 
% Syntax:
%   [ signal_names_1D, num_signals, batch_name ] = ...
%       ardupilotInitLogMuxerCore( block_path, num_signals_in, batch_name_in )
% 
% Inputs:
%   Take a look at ardupilotInitLogMuxer and ardupilotInitLogMuxerCore
% 
% Outputs:
%   Take a look at ardupilotInitLogMuxer and ardupilotInitLogMuxerCore
% 
% See also:
%   ardupilotInitLogMuxer, ardupilotInitLogMuxerCore

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2022 Yannic Beyer
% *************************************************************************

num_char_per_signal = 3;

ph = get_param( block_path, 'PortHandles' );
lh = get_param(ph.Inport,'Line');

num_inports = length(lh);
num_signals = num_inports;

signal_names = uint8(ones(num_char_per_signal,num_inports));

num_signals_cumsum = cumsum([0,num_signals_vec]);

for j = 1:num_inports
    
    signal_idx = max(find(j>num_signals_cumsum));
    signal_number = j - num_signals_cumsum(signal_idx);
    if lh{j} == -1
        current_name = emptySignalName(signal_number);
    else
        current_name = get_param(lh{j},'Name');
        if isempty(current_name)
            current_name = emptySignalName(signal_number);
        end
    end
    
    current_name = removeSpecialChars( current_name );

    current_name_length = length(current_name);
    i_end = min(num_char_per_signal,current_name_length);
    for i = 1:i_end
        signal_names(i,j) = current_name(i);
    end
end

signal_names_1D = signal_names(:);

    function signal_name = emptySignalName(i)
        signal_name = ['s',num2str(i)];
    end

    function current_name = removeSpecialChars( current_name )
        idx0 = uint8('0');
        idx9 = uint8('9');
        idxA = uint8('A');
        idxZ = uint8('Z');
        idxa = uint8('a');
        idxz = uint8('z');
        is_area_1 = current_name >= idx0 & current_name <= idx9;
        is_area_2 = current_name >= idxA & current_name <= idxZ;
        is_area_3 = current_name >= idxa & current_name <= idxz;
        current_name(~is_area_1 & ~is_area_2 & ~is_area_3) = [];
    end

end
