function [] = blockInitToggleEnable(is_enable_checked,varargin)
%blockInitToggleEnable toggle Enable block of Simulink library block
%   This function should be called from the initialization of a Simulink
%   block.
%   It adds an "Enable" block if input "is_enable_checked" is true or
%   removes the "Enable" block if "is_enable_checked" is false.
% 
% Syntax:
%   blockInitToggleEnable(is_enable_checked)
%   blockInitToggleEnable(is_enable_checked,states_when_enabling)
% 
% Inputs:
%   is_enable_checked           Simulink block mask parameter which
%                               controls whether the block should be an
%                               "Enabled Subsystem or not (bool)
%   states_when_enabling        Simulink block mask parameter which
%                               controls whether the initialization of the
%                               block should be "held" (1) or "reset" (2),
%                               default: 1 (double or int)

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2023 Yannic Beyer
%   Copyright (C) 2023 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

if isempty(varargin)
    states_when_enabling = 1;
else
    states_when_enabling = varargin{1};
end

if states_when_enabling == 1
    states_when_enabling_str = 'held';
elseif states_when_enabling == 2
    states_when_enabling_str = 'reset';
end

block_list = find_system(gcb,'LookUnderMasks','on','FollowLinks','on',...
    'Name','Enable');
if is_enable_checked
    if isempty(block_list)
        add_block('built-in/EnablePort',[gcb,'/Enable']);
        block_list = find_system(gcb,'LookUnderMasks','on','FollowLinks','on',...
            'Name','Enable');
    end
    set_param(block_list{1},'StatesWhenEnabling',states_when_enabling_str);
else
    if ~isempty(block_list)
        delete_block([gcb,'/Enable']);
    end
end

end