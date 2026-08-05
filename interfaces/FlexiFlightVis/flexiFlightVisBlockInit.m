function [ffv,ffv_double_array,ffv_double_idx] = flexiFlightVisBlockInit(block_path,wing_cell,fuse_cell)
% flexiFlightVisBlockInit initialization function for Simulink library
% block flexiFlightVis_lib/Send to FlexiFlightVis

num_wings = length(wing_cell);
num_fuse = length(fuse_cell);

block_path_split = strsplit(block_path,'/');
model_name = block_path_split{1};
if strcmp(get_param(model_name,"FastRestart"),"off")
    flexiFlightVisInputFormat(block_path,num_wings,num_fuse);
end

[ffv,ffv_double_array,ffv_double_idx] = flexiFlightVisProtocol( wing_cell, fuse_cell );

end

function flexiFlightVisInputFormat(block_path,num_wings,num_fuse)
% Add as many inputs as desired and connect them to the bus assignment
% block

pos_shift_v = 60;
pos_shift_h = 130;
num_in_static = 2;
i1 = 1 + num_in_static;
    
% delete inport lines and signal conversions
lh = find_system(block_path,'LookUnderMasks','on','FollowLinks','on','SearchDepth',1,'FindAll','on','Type','Line');
for i = 1:length(lh)
    lh_block_type = get_param(get_param(lh(i),'SrcBlockHandle'),'BlockType');
    lh_block_name = get_param(get_param(lh(i),'SrcBlockHandle'),'Name');
    dst_block_name = get_param(get_param(lh(i),'DstBlockHandle'),'Name');
    if strcmp(lh_block_type,'Inport') && ~strcmp(lh_block_name,'rigid_body') && ~strcmp(lh_block_name,'config')
        delete_line(lh(i));
        lh_tvb = get_param([block_path,'/',dst_block_name],'LineHandles');
        delete_block([block_path,'/',dst_block_name]);
        delete_line(lh_tvb.Outport);
    end
end

inport_block_cell = find_system(block_path,'LookUnderMasks','on','FollowLinks','on','SearchDepth',1,'BlockType','Inport');

% sort inport blocks
for i = i1:length(inport_block_cell)
    set_param(inport_block_cell{i},'Port',num2str(i));
    set_param(inport_block_cell{i},'Name',['In',num2str(i)]);
end

inport_block_cell = find_system(block_path,'LookUnderMasks','on','FollowLinks','on','SearchDepth',1,'BlockType','Inport');

num_inports = length(inport_block_cell);

num_in_opt = num_inports - num_in_static;

num_inports_des = num_wings + num_fuse + num_in_static;

assigned_signals = 'rigid_body,config';

num_iter = max(num_inports,num_inports_des);

inport_names = {};

for i = i1:num_iter
    
    j = i - num_in_static;
    
    if j <= num_wings
        inport_names{j} = ['wing_',num2str(j),'_state'];
    else
        inport_names{j} = ['fuselage_',num2str(j-num_wings),'_state'];
    end
    
    if j > num_in_opt && i <= num_inports_des
        inport_block_cell{i} = [block_path,'/',inport_names{j}];
        add_block('simulink/Sources/In1',inport_block_cell{i});
        if j > 1
            pos_prev = get_param([block_path,'/',inport_names{j-1}],'Position');
        else
            pos_prev = get_param(inport_block_cell{2},'Position');
        end
        pos_new = [pos_prev+pos_shift_v*[0,1,0,1]];
        set_param(inport_block_cell{i},'Position',pos_new);
    end
    if i > num_inports_des
        delete_block(inport_block_cell{i});
    else
        set_param(inport_block_cell{i},'Port',num2str(i));
        set_param(inport_block_cell{i},'Name',inport_names{j});
    end

end

for i = i1:num_inports_des
    j = i - num_in_static;
    assigned_signals = [assigned_signals,',',inport_names{j}];
end

set_param([block_path,'/Bus Assignment'],'AssignedSignals',assigned_signals);

% connect inports with bus creator
for i = i1:num_inports_des
    j = i - num_in_static;
    tvb_name = ['To Virtual Bus',num2str(i)];
    tvb_path = [block_path,'/',tvb_name];
    add_block('simulink/Signal Attributes/Signal Conversion',tvb_path);
    set_param(tvb_path,'ConversionOutput','Virtual bus');
    inport_pos = get_param([block_path,'/',inport_names{j}],'Position');
    pos_tvb = inport_pos + pos_shift_h*[1,0,1,0] + [0,-15,10,15];
    set_param(tvb_path,'Position',pos_tvb);
    add_line(block_path,[inport_names{j},'/1'],[tvb_name,'/1']);
    add_line(block_path,[tvb_name,'/1'],['Bus Assignment/',num2str(i+1)]);
end

end
