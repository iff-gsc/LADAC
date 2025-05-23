% *** configuration script for the Taranis_X9D joystick ***

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
%
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% values for all channels when the sticks are centered
jystck.ch_trim = [0 0 0 0 -1 -1 -1 0 0 0 0 0 0 0 0 0]';
% index of all channels so that they are compatible with the Simulink block
jystck.ch_idx = uint8([2 3 1 4 5 6 7 8]');
% logical array that is true if a channel is reversed
jystck.ch_reversed = logical([0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]');
% maximum channel values
jystck.ch_max = [1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1]';
% minimum channel values
jystck.ch_min = [-1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1]';
% maximum dead zone values of the channels
jystck.ch_dead_max = [0 0 0 0 0 0 0.01 0 0 0 0 0 0 0 0 0]';
% minimum dead zone values of the channels
jystck.ch_dead_min = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]';
