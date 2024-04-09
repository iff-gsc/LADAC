function [] = vonKarman2DInitBlock( path, len )
% vonKarman2DInitBlock initialize Simulink block "2D Von Karman Turbulence"
% This function is called at the initialization of the Simulink block.
% 
% Syntax:
%   vonKarman2DInitBlock( path, len )
% 
% Inputs:
%   path                Simulink path of the "2D Von Karman Turbulence"
%                       block (when called from block initialization: gcb)
%   len                 Array which contains the number of lateral
%                       positions of interest
% 
% Outputs:
%   -
% 
% See also:
%   vonKarman2D

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2023 Yannic Beyer
%   Copyright (C) 2023 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

num_outports = length(len);

if num_outports < 1
    error('Lateral positions of interest must be a cell array of length 1 or larger.');
end

inport_length = sum(len);

index_begin = [1,cumsum(len(1:end-1))+1];
index_end = cumsum(len);

matlab_fnc_block = find_system(path,'LookUnderMasks','on','FollowLinks','on','BlockType','SubSystem');
matlab_fnc_name = get_param(matlab_fnc_block{2},'Name');
selector_blocks = find_system(path,'LookUnderMasks','on','FollowLinks','on','BlockType','Selector');
outport_blocks = find_system(path,'LookUnderMasks','on','FollowLinks','on','BlockType','Outport');
% Remove Matlab function block output
outport_blocks(1) = [];

for i = 1:max(num_outports,length(selector_blocks))
    sel_name{i} = ['Selector',num2str(i)];
    out_name{i} = ['w',num2str(i)];
    sel_path{i} = [path,'/',sel_name{i}];
    out_path{i} = [path,'/',out_name{i}];
end

pos_sel_1 = get_param(selector_blocks{1},'Position');
pos_out_1 = get_param(outport_blocks{1},'Position');
Delta_pos = 50;

for i = num_outports+1:length(selector_blocks)
    delete_line(path,[sel_name{i},'/1'],[out_name{i},'/1']);
    delete_line(path,[matlab_fnc_name,'/1'],[sel_name{i},'/1']);
    delete_block(selector_blocks{i});
    delete_block(outport_blocks{i});
end

for i = length(selector_blocks)+1:num_outports
    add_block('simulink/Signal Routing/Selector',sel_path{i});
    add_block('built-in/Outport',out_path{i});
    set_param(sel_path{i},'Position',pos_sel_1 + [0,1,0,1]*Delta_pos*(i-1) );
    set_param(out_path{i},'Position',pos_out_1 + [0,1,0,1]*Delta_pos*(i-1) );
    add_line(path,[sel_name{i},'/1'],[out_name{i},'/1']);
    add_line(path,[matlab_fnc_name,'/1'],[sel_name{i},'/1']);
end

for i = 1:num_outports
    set_param(sel_path{i},'IndexParamArray',{'1'});
    set_param(sel_path{i},'InputPortWidth',num2str(inport_length));
    set_param(sel_path{i},'IndexParamArray',{[num2str(index_begin(i)),':',num2str(index_end(i))]});
end

end
