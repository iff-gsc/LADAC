 % *** configuration script for the HORUS X12 joystick ***
 % Channel 8 not recognized on Ubuntu 24.04
 % Channel 5 and 6 are switched on Windows 11

 % values for all channels when the sticks are centered
jystck.ch_trim = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]';
 % index of all channels so that they are compatible with the Simulink block
jystck.ch_idx = uint8([1 2 3 4 5 6 7 8]');
 % logical array that is true if a channel is reversed
jystck.ch_reversed = logical([0 0 0 0 1 1 1 1 0 0 0 0 0 0 0 0]');
 % maximum channel values
jystck.ch_max = [0.98 0.99 0.98 0.98 1 1 1 1 1 1 0 1 1 1 1 1]';
 % minimum channel values
jystck.ch_min = [-0.98 -0.98 -0.98 -0.99 -1 -1 -1 -1 -1 -1 0 -1 -1 -1 -1 -1]';
 % maximum dead zone values of the channels
jystck.ch_dead_max = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]';
 % minimum dead zone values of the channels
jystck.ch_dead_min = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]';
