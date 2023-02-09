function [] = cntrlBlockInit(is_enable_checked,states_when_enabling)
% cntrlBlockInit initialization of controller modules Simulink blocks
%   For some controller modules blocks it makes sense to include an option
%   make an "Enabled Subsystem" of them and to define how the "Enabled
%   Subsystem" is initialized after enabling (held or reset).
%   This block should be called from the initialization of a Simulink
%   block.
% 
% Syntax:
%   cntrlBlockInit(is_enable_checked,states_when_enabling)
% 
% Inputs:
%   is_enable_checked           Simulink block mask parameter which
%                               controls whether the block should be an
%                               "Enabled Subsystem or not (bool)
%   states_when_enabling        Simulink block mask parameter which
%                               controls whether the initialization of the
%                               block should be "held" (1) or "reset" (2)
%                               (double or int)
% 
% See also:
%   blockInitToggleEnable

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2023 Yannic Beyer
%   Copyright (C) 2023 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

mask_names = get_param(gcb,'MaskNames');
mask_enables = get_param(gcb,'MaskEnables');

for i = 1:length(mask_names)
    if strcmp(mask_names{i},'states_when_enabling')
        if is_enable_checked
            mask_enables{i} = 'on';
        else
            mask_enables{i} = 'off';
        end
    end
end

set_param(gcb,'MaskEnables',mask_enables);  

blockInitToggleEnable(is_enable_checked,states_when_enabling)

end