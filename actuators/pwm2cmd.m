% pwm2cmd converts a command in the range of 1000~2000 into a range of
%   0~1. 1000 can either be mapped to 0 or to 1 depending on the
%   is_reversed input.
% 
% Inputs:
%   u_PWM           Vector of commands in the range of 1000~2000.
%   is_reversed     Vector with logicals of the same size as u_PWM. 
%                   True means that 0->2000 and 1->1000.
%                   False means that 0->1000 and 1->2000.
% 
% Outputs:
%   u_01            Vector of commands in the range of 0~1.

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

function u_01 = pwm2cmd(u_PWM,is_reversed) %codegen

    reverse_factor = ones(size(is_reversed));
    reverse_factor(is_reversed) = -1;

    u_01 = 0.5 + ( u_PWM - 1500 ) / 1000 .* reverse_factor;

end