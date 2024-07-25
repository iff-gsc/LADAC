function quadcopterBlockInit(is_external_force_checked, is_external_torque_checked)
% QUADCOPTERBLOCKINIT
%   Initialize function for 'Quadcopter' in 'quadcopter_lib'
% 
% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
% 	Copyright (C) 2024 Jonas Withelm
%   Copyright (C) 2024 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

blk_p = gcb;

%% Inports
port_idx = 3;

block_name = 'R_b';
blockInitToggleInport(block_name, is_external_force_checked);
if is_external_force_checked
    block_path = [blk_p '/' block_name];
    set_port_idx( block_path );
end

block_name = 'Q_b';
blockInitToggleInport(block_name, is_external_torque_checked);
if is_external_torque_checked
    block_path = [blk_p '/' block_name];
    set_port_idx( block_path );
end





%% NESTED FUNCTIONS
function set_port_idx(block_path)
    set_param( block_path, 'Port', num2str(port_idx) );
    port_idx = port_idx + 1;
end

end