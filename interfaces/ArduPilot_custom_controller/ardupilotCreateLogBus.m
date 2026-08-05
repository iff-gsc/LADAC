function [log_config] = ardupilotCreateLogBus()
% ardupilotCreateLogBus create Simulink bus object "logConfigBus" for ArduPilot logs
%   ArduPilot logs can be configured using the Simulink block "log muxer".
%   The output of that block is an array of nonvirtual buses. For these
%   nonvirtual buses the data type must be provided. That is why this
%   function exists and why we need the variable "logConfigBus" in the base
%   workspace.
%   Fortunately, the "logConfigBus" will automatically be created during the
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
%   log_config              struct corresponding to the Simulink bus object
%                           "logConfigBus" that will be assigned to the
%                           base workspace
% 
% See also:
%   ardupilotInitLogMuxer, ardupilotCreateInputBuses

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2022 Yannic Beyer
% *************************************************************************

num_char_per_signal         = 3;
num_char_per_batch          = 4;
num_signals_per_batch_max   = 14;

max_label_char_length   = num_signals_per_batch_max * num_char_per_signal;
label_char_length       = max_label_char_length;

log_config.num_signals 	= uint8(0);
log_config.signal_names	= uint8(zeros(label_char_length,1));
log_config.batch_name	= uint8(zeros(num_char_per_batch,1));

struct2bus( log_config, 'logConfigBus' );

end
