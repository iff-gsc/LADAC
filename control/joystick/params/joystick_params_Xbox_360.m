% *** configuration script for the Xbox_360 joystick ***

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
%
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% values for all channels when the sticks are centered
jystck.ch_trim = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]';
% index of all channels so that they are compatible with the Simulink block
jystck.ch_idx = uint8([4 5 2 1 3 13 14 9]');
% logical array that is true if a channel is reversed
jystck.ch_reversed = logical([0 1 0 0 1 0 0 0 0 0 0 0 0 0 0 0]');
% maximum channel values
jystck.ch_max = [1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1]';
% minimum channel values
jystck.ch_min = [-1 -0.97 0 -1 -0.93 -1 -1 -1 0 -1 -1 -1 0 0 -1 -1]';
% maximum dead zone values of the channels
jystck.ch_dead_max = [0.11 0.1 0 0.07 0.12 0 0 0 0 0 0 0 0 0 0 0]';
% minimum dead zone values of the channels
jystck.ch_dead_min = [-0.07 -0.02 0 -0.06 -0.07 0 0 0 0 0 0 0 0 0 0 0]';
