function [] = pt2BlockInit(param_src, y_0_src, y_dt_0_src, is_y_dt_checked, is_y_dt2_checked, is_external_reset_checked)
% PT2BLOCKINIT  Initialize function for 'PT2 discrete with saturation' in
% 'filters_lib'
% 
% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
% 	Copyright (C) 2022 Yannic Beyer
% 	Copyright (C) 2024 Jonas Withelm
%   Copyright (C) 2024 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************


blk_p = gcb;

switch param_src
    case 1
        is_omega_external = false;
        is_d_external = false;
    case 2
        is_omega_external = true;
        is_d_external = true;
end
switch y_0_src
    case 1
        is_y_0_external = false;
    case 2
        is_y_0_external = true;
end
switch y_dt_0_src
    case 1
        is_y_dt_0_external = false;
    case 2
        is_y_dt_0_external = true;
end

mask_names = get_param(blk_p,'MaskNames');
mask_enables = get_param(blk_p,'MaskEnables');


%% Inports
port_idx = 2;

block_name = 'y_0';
blockInitReplaceBlock( block_name, is_y_0_external, 'Constant', 'Inport' );
block_path = [blk_p '/' block_name];
if ~is_y_0_external
    set_param( block_path, 'Value', 'x_init' );
else
    set_port_idx( block_path );
end

block_name = 'y_dt_0';
blockInitReplaceBlock( block_name, is_y_dt_0_external, 'Constant', 'Inport' );
block_path = [blk_p '/' block_name];
if ~is_y_dt_0_external
    set_param( block_path, 'Value', 'x_dt_init' );
else
    set_port_idx( block_path );
end

block_name = 'omega';
blockInitReplaceBlock( block_name, is_omega_external, 'Constant', 'Inport' );
block_path = [blk_p '/' block_name];
if ~is_omega_external
    set_param( block_path, 'Value', 'omega' );
else
    set_port_idx( block_path );
end

block_name = 'd';
blockInitReplaceBlock( block_name, is_d_external, 'Constant', 'Inport' );
block_path = [blk_p '/' block_name];
if ~is_d_external
    set_param( block_path, 'Value', 'd' );
else
    set_port_idx( block_path );
end

block_name = 'external_reset';
blockInitReplaceBlock( block_name, is_external_reset_checked, 'Ground', 'Inport' );
if is_external_reset_checked
    block_path = [blk_p '/' block_name];
    set_port_idx( block_path );
end


% Toggle mask parameters enabled state
for i = 1:length(mask_names)
    if strcmp(mask_names{i},'x_init')
        if is_y_0_external
            mask_enables{i} = 'off';
        else
            mask_enables{i} = 'on';
        end
    elseif strcmp(mask_names{i},'x_dt_init')
        if is_y_dt_0_external
            mask_enables{i} = 'off';
        else
            mask_enables{i} = 'on';
        end
    elseif strcmp(mask_names{i},'omega')
        if is_omega_external
            mask_enables{i} = 'off';
        else
            mask_enables{i} = 'on';
        end
    elseif strcmp(mask_names{i},'d')
        if is_d_external
            mask_enables{i} = 'off';
        else
            mask_enables{i} = 'on';
        end
    end
end
set_param(blk_p,'MaskEnables',mask_enables);  


%% Outports
port_idx = 2;

block_name = 'y_dt';
blockInitToggleOutport(block_name, is_y_dt_checked);
if is_y_dt_checked
    block_path = [blk_p '/' block_name];
    set_port_idx( block_path );
end

block_name = 'y_dt2';
blockInitToggleOutport(block_name, is_y_dt2_checked);
if is_y_dt2_checked
    block_path = [blk_p '/' block_name];
    set_port_idx( block_path );
end





%% NESTED FUNCTIONS
function set_port_idx(block_path)
    set_param( block_path, 'Port', num2str(port_idx) );
    port_idx = port_idx + 1;
end

end