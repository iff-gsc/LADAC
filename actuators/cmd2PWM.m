function u_PWM = cmd2PWM(u_01,is_reversed)
% cmd2PWM converts a command in the range of 0~1 into a range of
%   1000~2000. 0 can either be mapped to 1000 or to 2000 depending on the
%   is_reversed input.
%
% Inputs:
%   u_01            Vector of commands in the range of 0~1.
%   is_reversed     Vector with logicals of the same size as u_01.
%                   True means that 0->2000 and 1->1000.
%                   False means that 0->1000 and 1->2000.
%
% Outputs:
%   u_PWM           Vector of commands in the range of 1000~2000.
%

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

reverse_factor = ones(size(is_reversed));
reverse_factor(is_reversed) = -1;

u_PWM = 1500 + ( u_01 - 0.5 ) * 1000 .* reverse_factor;

end