function flightgearQuadcopterBlockInit(is_susp_payload)
% FLIGHTGEARQUADCOPTERBLOCKINIT
%   Initialize function for 'Flight Gear Animation (Quadcopter)' in
%   'flightgear_visualization_lib'
% 
% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
% 	Copyright (C) 2024 Jonas Withelm
%   Copyright (C) 2024 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

blk_p = gcb;

%% Inports
port_idx = 6;

block_names = {'EulerAngles PLD'; ...
               'rod_length PLD'; ...
               'rod_diameter PLD'; ...
               'payload_diameter PLD'; ...
               'joint_pos PLD'};

for idx = 1:numel(block_names)
    block_name = block_names{idx};
    blockInitToggleInport(block_name, is_susp_payload);
    if is_susp_payload
        block_path = [blk_p '/' block_name];
        set_port_idx( block_path );
    end
end





%% NESTED FUNCTIONS
function set_port_idx(block_path)
    set_param( block_path, 'Port', num2str(port_idx) );
    port_idx = port_idx + 1;
end

end