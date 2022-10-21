function [] = filtersBlockInit(y_0_src,y_dt_0_src,param_src,...
    is_y_dt_checked,is_y_dt2_checked)

switch y_0_src
    case 1
        is_y_0_checked = false;
    case 2
        is_y_0_checked = true;
end
switch y_dt_0_src
    case 1
        is_y_dt_0_checked = false;
    case 2
        is_y_dt_0_checked = true;
end
switch param_src
    case 1
        is_params_external = false;
    case 2
        is_params_external = true;
end

mask_names = get_param(gcb,'MaskNames');
mask_enables = get_param(gcb,'MaskEnables');

block_name = 'y_0';
blockInitReplaceBlock( block_name, is_y_0_checked, 'Constant', 'Inport' );
block = find_system(gcb,'LookUnderMasks','on',...
            'FollowLinks','on','SearchDepth',1,'Name',block_name);
if ~is_y_0_checked
    set_param( block{1}, 'Value', 'x_init' );
else
    set_param( block{1}, 'Port', '2' );
end

block_name = 'y_dt_0';
blockInitReplaceBlock( block_name, is_y_dt_0_checked, 'Constant', 'Inport' );
block = find_system(gcb,'LookUnderMasks','on',...
            'FollowLinks','on','SearchDepth',1,'Name',block_name);
if ~is_y_dt_0_checked
    set_param( block{1}, 'Value', 'x_dt_init' );
else
    set_param( block{1}, 'Port', '2' );
    if is_y_0_checked
        set_param( block{1}, 'Port', '3' );
    end
end

block_name = 'omega';
blockInitReplaceBlock( block_name, is_params_external, 'Constant', 'Inport' );
if ~is_params_external
    block = find_system(gcb,'LookUnderMasks','on',...
                'FollowLinks','on','SearchDepth',1,'Name',block_name);
    set_param( block{1}, 'Value', 'omega' );
end

block_name = 'd';
blockInitReplaceBlock( block_name, is_params_external, 'Constant', 'Inport' );
if ~is_params_external
    block = find_system(gcb,'LookUnderMasks','on',...
                'FollowLinks','on','SearchDepth',1,'Name',block_name);
    set_param( block{1}, 'Value', 'd' );
end

for i = 1:length(mask_names)
    if strcmp(mask_names{i},'x_init')
        if is_y_0_checked
            mask_enables{i} = 'off';
        else
            mask_enables{i} = 'on';
        end
    elseif strcmp(mask_names{i},'x_dt_init')
        if is_y_dt_0_checked
            mask_enables{i} = 'off';
        else
            mask_enables{i} = 'on';
        end
    elseif strcmp(mask_names{i},'omega')
        if is_params_external
            mask_enables{i} = 'off';
        else
            mask_enables{i} = 'on';
        end
    elseif strcmp(mask_names{i},'d')
        if is_params_external
            mask_enables{i} = 'off';
        else
            mask_enables{i} = 'on';
        end
    end
end

set_param(gcb,'MaskEnables',mask_enables);  

block_name = 'y_dt';
blockInitToggleOutport(block_name,is_y_dt_checked);
if is_y_dt_checked
    block = find_system(gcb,'LookUnderMasks','on',...
            'FollowLinks','on','SearchDepth',1,'Name',block_name);
    set_param( block{1}, 'Port', '2' );
end
blockInitToggleOutport('y_dt2',is_y_dt2_checked);

end