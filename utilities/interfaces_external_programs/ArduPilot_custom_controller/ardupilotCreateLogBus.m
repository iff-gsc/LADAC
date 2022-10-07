function [] = ardupilotCreateLogBus(varargin)
% ardupilotCreateLogBus create Simulink bus object "logBus" for ArduPilot logs
%   ArduPilot logs can be configured using the Simulink block "log muxer".
%   The output of that block is an array of nonvirtual buses. For these
%   nonvirtual buses the data type must be provided. That is why this
%   function exists and why we need the variable "logBus" in the base
%   workspace.
%   Fortunately, the "logBus" will automatically be created during the
%   initialization of the Simulink block "log muxer". For more information
%   have a look at the "log muxer" block.
% 
% Syntax:
%   ardupilotCreateLogBus()
% 
% Inputs:
%   -
% 
% Outputs:
%   none (but a Simulink bus object named "logBus" will be assigned to the
%   base workspace)
% 
% See also:
%   ardupilotInitLogMuxer, ardupilotInitLogMuxerCore

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2022 Yannic Beyer
% *************************************************************************

if isempty(varargin)
    data_type = 'single';
else
    data_type = varargin{1};
end

num_char_per_signal         = 3;
num_char_per_batch          = 4;
num_signals_per_batch_max   = 14;

max_label_char_length   = num_signals_per_batch_max * num_char_per_signal;
label_char_length       = max_label_char_length;

log.signals        = zeros(num_signals_per_batch_max,1);
log.num_signals    = uint8(0);
log.signal_names   = uint8(zeros(label_char_length,1));
log.batch_name     = uint8(zeros(num_char_per_batch,1));


if strcmp(data_type,'single')
    log = structDouble2Single(log);
end

struct2bus( log, 'logBus' );

end
