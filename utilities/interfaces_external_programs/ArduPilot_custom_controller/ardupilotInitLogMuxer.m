function [] = ardupilotInitLogMuxer( block_path, batch_name, num_signals,varargin )
% ardupilotInitLogMuxer initialize ArduPilot Log Muxer Simulink block
%   This function is called from the initialization of the Simulink block
%   log muxer.
%   It dynamically configures the block mask so that a variable number of
%   inports can be provides. Moreover, the portnames are adjusted
%   automatically, either from the input signal name or with default names
%   if no signal names are specified. It also creates a logBus bus object
%   in the base workspace.
% 
% Syntax:
%   ardupilotInitLogMuxer( gcb, batch_name, num_signals )
%   ardupilotInitLogMuxer( gcb, batch_name, num_signals, data_type )
% 
% Inputs:
%   Take a look at the mask of the "log muxer" Simulink block.
%   data_type           (optional) logBus data type ('single' or 'double')
% 
% Outputs:
%   -
% 
% See also:
%   ardupilotCreateLogBus, ardupilotInitLogMuxerCore,
%   ardupilotCreateLogSignalNames

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2022 Yannic Beyer
% *************************************************************************

num_signals_per_batch_max = 14;
    
ardupilotCreateLogBus(varargin{:});

num_batches_max = length(batch_name);

default_log_names = ardupilotDefaultLogNames(num_signals_per_batch_max);

ardupilotResetLogMuxer(block_path,batch_name);
for i = 1:num_batches_max
    for j = 1:num_signals_per_batch_max
        inport_name = [batch_name{i},'.',default_log_names{j}];
        if j <= num_signals(i)
            is_enabled = true;
        else
            is_enabled = false;
        end
        blockInitToggleInport( inport_name, is_enabled );
        if is_enabled
            inport_path = find_system(block_path,'LookUnderMasks','on','FollowLinks','on','SearchDepth',1,'Name',inport_name);
            if i == 1
                port_number = j;
            else
                port_number =  sum(num_signals(1:i-1)) + j;
            end
            set_param(inport_path{1},'Port',num2str(port_number));
        end
    end
end

[~,signal_names] = ardupilotCreateLogSignalNames( block_path, num_signals );
num_signals_total = sum(num_signals);
num_signals_cumsum = cumsum([0,num_signals]);
for i = 1:num_signals_total
    batch_idx = max(find(i>num_signals_cumsum));
    inport_name = [char(batch_name(batch_idx)'),'.',char(signal_names(:,i)')];
    inport_name(inport_name==1)=[];
    blockInitRenameInport( i, inport_name );
end

end


function [] = ardupilotResetLogMuxer( block_path, batch_name )
% set default names for all source blocks

num_batches = length(batch_name);

subsys_block_names = repmat({'log muxer core'},num_batches,1);
for ii = 1:num_batches
    subsys_block_names{ii}(end+1) = num2str(ii);
end

for ii = 1:num_batches
    subsys_block_cell = find_system(block_path,'LookUnderMasks','on','FollowLinks','on','SearchDepth',1,'Name',subsys_block_names{ii});

    if isempty(subsys_block_cell)
        error('ardupilotResetLogMuxer: Block is not named correctly.')
    else
        subsys_block_path = subsys_block_cell{1};
    end

    all_ports = get_param(subsys_block_path, 'PortConnectivity');

    src_block_handles = [all_ports(1:end-1).SrcBlock];
    
    num_signals_per_batch_max = length(all_ports)-1;

    default_log_names_ = ardupilotDefaultLogNames(num_signals_per_batch_max);

    for jj = 1:length(src_block_handles)
        src_block_name = [batch_name{ii},'.',default_log_names_{jj}];
        set_param(src_block_handles(jj),'Name',src_block_name);
    end
end

end


function default_log_names = ardupilotDefaultLogNames(last_signal_number)
default_log_names = {};
for i = 1:last_signal_number
    default_log_names{i} = ['s',num2str(i)];
end
end
