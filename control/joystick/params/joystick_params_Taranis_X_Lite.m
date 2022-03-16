% *** configuration script for the Taranis_X_Lite joystick ***

% This configuration file only applies to specific model settings on
% the transmitter! Generate your own joystick configuration if it
% does not work for you. For more details look at the joystick README.

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
%
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% values for all channels when the sticks are centered
jystck.ch_trim = [0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0]';
% index of all channels so that they are compatible with the Simulink block
jystck.ch_idx = uint8([2 3 1 4 5 11 12 13]');
% logical array that is true if a channel is reversed
jystck.ch_reversed = logical([0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]');
% maximum channel values
jystck.ch_max = [1 1 1 1 1 1 1 1 1 1 0 0 0 1 1 1]';
% minimum channel values
jystck.ch_min = [-1 -1 -1 -1 -1 -1 -1 -1 -1 -1 0 0 0 -1 -1 -1]';
% maximum dead zone values of the channels
jystck.ch_dead_max = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]';
% minimum dead zone values of the channels
jystck.ch_dead_min = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]';
