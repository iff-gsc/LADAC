function [] = rigidBodyBlockInit(is_g_checked)
%rigidBodyBlockInit init block "State equations (Quaternions)"
%   If block check box is enabled, create new block inport or outport.
%   If block check box is disabled, delete new block inport or outport.

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2023 Yannic Beyer
%   Copyright (C) 2023 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

blockInitReplaceBlock( 'g', is_g_checked, 'Constant', 'Inport' );

mask_names = get_param(gcb,'MaskNames');
mask_enables = get_param(gcb,'MaskEnables');

if ~is_g_checked
	set_param([gcb,'/g'],'Value','g');
end

for i = 1:length(mask_names)
    if strcmp(mask_names{i},'g')
        if is_g_checked
            mask_enables{i} = 'off';
        else
            mask_enables{i} = 'on';
        end
    end
end

set_param(gcb,'MaskEnables',mask_enables);  

end