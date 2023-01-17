function [] = ardupilotInitLogMuxer( block_path, batch_name, num_signals )
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
%   ardupilotInitLogMuxer( block_path, batch_name, num_signals )
% 
% Inputs:
%   Take a look at the mask of the "log muxer" Simulink block.
%   data_type           (optional) logBus data type ('single' or 'double')
% 
% Outputs:
%   -
% 
% See also:
%   ardupilotCreateLogBus, ardupilotCreateLogSignalNames

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2022 Yannic Beyer
% *************************************************************************

persistent memory;

num_batches_max = 5;

inport_path = find_system(block_path,'LookUnderMasks','on','FollowLinks','on','SearchDepth',1,'BlockType','Inport');
num_inports = length(inport_path);

if ~isempty(memory)
    if num_inports ~= sum(memory.num_signals)
        memory = [];
    end
end

if isempty(memory)
    batch_name_init = cell(1,num_batches_max);
    num_signals_init = zeros(1,num_batches_max);
    idx_batch = 1;
    for i = 1:num_inports
        [~,b]=fileparts(inport_path{i});
        if i == 1
            batch_name_init{1} = b;
            num_signals_init(1) = 1;
        else
            if ~strcmp(b,batch_name_init{idx_batch})
                idx_batch = idx_batch + 1;
            end
            batch_name_init{idx_batch} = b;
            num_signals_init(idx_batch) = num_signals_init(idx_batch) + 1;
        end
    end
    memory.batch_name = batch_name_init;
    memory.num_signals = num_signals_init;
end

log = ardupilotCreateLogBus();

num_signals_cum = cumsum(num_signals);

mux_block_cell = find_system(block_path,'LookUnderMasks','on','FollowLinks','on','SearchDepth',1,'BlockType','Mux');

line_handles = get_param(mux_block_cell{1},'LineHandles');
input_line_handles = line_handles.Inport;

% delete all lines to avoid if, else, ...
deleteLines(input_line_handles);

% port names: update batch name
inport_path = updateBatchNames(inport_path,batch_name,memory);

% create all ports
inport_path = addRemovePorts(block_path,inport_path,batch_name,num_signals,memory);

% set mux
set_param(mux_block_cell{1},'Inputs',num2str(num_signals_cum(end)));

% place inports next to mux inputs and connect them
positionConnectInports(mux_block_cell{1},inport_path)

[~,signal_names] = ardupilotCreateLogSignalNames( block_path, num_signals );

setInportNames(num_signals,signal_names,batch_name);

idx_start = cumsum([1,num_signals]);
idx_end = cumsum(num_signals);
log_config = repmat(log,1,num_batches_max);
for i = 1:num_batches_max
    log_config(i).num_signals(:) = uint8(num_signals(i));
    if num_signals(i)>0
        signal_names_i = uint8(signal_names(:,idx_start(i):idx_end(i)));
        log_config(i).signal_names(1:numel(signal_names_i)) = signal_names_i;
    end
    batch_name_i = uint8(batch_name{i});
    log_config(i).batch_name(1:length(batch_name_i)) = batch_name_i;
end

% avoid data copying in generated code
log_config_param = Simulink.Parameter(log_config);
log_config_param.CoderInfo.StorageClass = 'Custom';
log_config_param.CoderInfo.CustomStorageClass = 'Const';

assignin('base','log_config',log_config_param);

memory.batch_name = batch_name;
memory.num_signals = num_signals;

end

function [] = deleteLines(input_line_handles)
for i = 1:length(input_line_handles)
    if input_line_handles(i) ~= -1
        delete_line(input_line_handles(i));
    end
end
end

function port_number = portNumber(num_signals_cum_0,i,j)
port_number = num_signals_cum_0(i) + j;
end

function [inport_path] = updateBatchNames(inport_path,batch_name,memory)
num_batches_max = 5;
num_signals_cum = cumsum(memory.num_signals);
num_signals_cum_0 = [0,num_signals_cum];
for i = 1:num_batches_max
    for j = 1:memory.num_signals(i)
        pn = portNumber(num_signals_cum_0,i,j);
        port_name = get_param(inport_path{pn},'Name');
        port_name_new = strrep(port_name,memory.batch_name{i},batch_name{i});
        set_param(inport_path{pn},'Name',port_name_new);
        inport_path(pn) = strrep(inport_path{pn},memory.batch_name(i),batch_name(i));
    end
end
end

function [inport_path] = addRemovePorts(block_path,inport_path,batch_name,num_signals,memory)
num_batches_max = 5;
num_signals_cum = cumsum(num_signals);
num_signals_cum_0 = [0,num_signals_cum];
diff_num_signals = num_signals - memory.num_signals;
for i = 1:num_batches_max
    if diff_num_signals(i) > 0
        for j = memory.num_signals(i)+1:num_signals(i)
            pn = portNumber(num_signals_cum_0,i,j);
            new_block_path = [block_path,'/',batch_name{i},'.','s',num2str(j)];
            add_block('simulink/Sources/In1',new_block_path);
            set_param(new_block_path,'Port',num2str(pn));
            inport_path(pn+1:end+1) = inport_path(pn:end);
            inport_path{pn}=new_block_path;
        end
    elseif diff_num_signals(i) < 0
        for j = memory.num_signals(i):-1:num_signals(i)+1
            pn = num_signals_cum_0(i) + j;
            delete_block(inport_path{pn});
            inport_path(pn) = [];
        end
    end
end
end

function [] = positionConnectInports(mux_path,inport_path)
Delta_pos_y = -200;
all_ports = get_param(mux_path, 'PortConnectivity');
input_pos = {all_ports(1:end-1).Position};
num_signals = length(input_pos);
inport_pos = get_param(inport_path{1},'position');
inport_width = inport_pos(3)-inport_pos(1);
inport_height = inport_pos(4)-inport_pos(2);
block_path = fileparts(mux_path);
for i = 1:num_signals
    pos_left = input_pos{i}(1)+Delta_pos_y;
    pos_top = input_pos{i}(2)-inport_height/2;
    pos_right = input_pos{i}(1)+inport_width+Delta_pos_y;
    pos_down = input_pos{i}(2)+inport_height/2;
    set_param( inport_path{i}, 'position', [ pos_left, pos_top, pos_right, pos_down ] );
    point_1_y = input_pos{i}(1)+Delta_pos_y+inport_width;
    point_1_x = input_pos{i}(2);
    point_2_y = input_pos{i}(1);
    point_2_x = input_pos{i}(2);
    add_line( block_path, [ point_1_y, point_1_x; point_2_y, point_2_x ] );
end
end

function [] = setInportNames(num_signals,signal_names,batch_name)
num_signals_total = sum(num_signals);
num_signals_cumsum = cumsum([0,num_signals]);
for i = 1:num_signals_total
    batch_idx = max(find(i>num_signals_cumsum));
    inport_name = [char(batch_name(batch_idx)'),'.',char(signal_names(:,i)')];
    inport_name(inport_name==1)=[];
    blockInitRenameInport( i, inport_name );
end
end
