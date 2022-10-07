function [ signal_names_1D, num_signals, batch_name ] = ...
    ardupilotInitLogMuxerCore( block_path, num_signals_in, batch_name_in )
% ardupilotInitLogMuxerCore initialize ArduPilot Log Muxer Core Simulink block
%   This function is called from the initialization of the Simulink block
%   log muxer core.
%   It automatically renames the portnames from the input signal
%   names. This Simulink block is usually located inside the Simulink block
%   "log muxer".
% 
% Syntax:
%   [signal_names_1D,num_signals,batch_name] = ardupilotInitLogMuxerCore(...
%       gcb,num_signals_in,batch_name_in)
% 
% Inputs:
%   Take a look at the mask of the "log muxer core" Simulink block and also
%   take a look at the "log muxer" block.
% 
% Outputs:
%   Take a look at the "log muxer core" Simulink block.
% 
% See also:
%   ardupilotInitLogMuxer, ardupilotCreateLogSignalNames

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2022 Yannic Beyer
% *************************************************************************

batch_name_len  = 4;
num_signals_max = 14;
num_signals     = uint8(num_signals_in);

batch_name = uint8(ones(1,batch_name_len));
i_end = min(batch_name_len,length(batch_name_in));
for i = 1:i_end
    batch_name(i) = uint8(batch_name_in(i));
end

[ signal_names_1D, signal_names ] = ardupilotCreateLogSignalNames( ...
    block_path, num_signals_max );

for i = 1:num_signals_max
    % don't use the exacte same inport names as in the parent model because
    % that causes signal disconnections using the "log muxer" block 
    % -> use '_' instead of '.'
    inport_name = [char(batch_name(:)'),'_',char(signal_names(:,i)')];
    inport_name(inport_name==1) = [];
    blockInitRenameInport( i, inport_name );
end

end
