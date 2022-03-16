function rpytm = joystickCh2Rpyt( channel, jystck )
% joystickCh2Rpyt converts the channel outputs of a joystick into roll,
%   pitch,yaw,throttle and 4 additional function values depending on the
%   parameters in the jystck struct. Roll, pitch, yaw and throttle are
%   defined according to [1].
%   This function considers two linear functions for positive and negative
%   stick inputs. The outputs are zero when the sticks are at their trim
%   positions. Moreover, the outputs are zero when the sticks are close to
%   their trim position and inside the dead zone specified in the jystck
%   struct.
% 
% Inputs:
%   channel           	Channel values of the joystick (16x1 array)
%   jystck              joystick struct with appropriate parameters for the
%                       used joystick (see joystickLoadParams)
% 
% Outputs:
%   rpytm               Roll, pitch, yaw, throttle and 4 additional
%                       functions (8x1 array) with values from -1 to 1 or
%                       0 to 1. Roll, pitch, yaw, throttle values are
%                       between -1 to 1.
%                       If zero throttle is defined to be stick down, the
%                       throttle output will be between 0 and 1.
%                       The 4 additional functions values are between 0 and
%                       1.
% 
% Literature:
%   [1] https://developer.dji.com/mobile-sdk/documentation/introduction/component-guide-remotecontroller.html
% 
% See also:
%   joystickLoadParams, joystickCalibrate

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% init
ch_range = zeros( size(jystck.ch_trim) );
ch_dead = zeros( size(jystck.ch_trim) );

% remove dead zone from throttle channel if stick down is zero throttle
if jystck.zero_throttle == -1
    throttle_idx = jystck.ch_idx(jystck.mode_idx(4));
    jystck.ch_dead_max(throttle_idx) = 0;
    jystck.ch_dead_min(throttle_idx) = 0;
end    

% reversed channels: convert logical array to array of 1 and -1
ch_sign = ones( size( jystck.ch_reversed ) );
ch_sign(jystck.ch_reversed) = -1;

% preliminary channel deflection from trim point
ch_delta = channel - jystck.ch_trim;

% distinguish between positive and negative channel outputs
ch_positive = ( channel - jystck.ch_trim ) > 0;

% are channels inside dead zone?
ch_is_dead = ch_delta<jystck.ch_dead_max & ch_delta>jystck.ch_dead_min;

% set channel deflection to zero if inside dead zone
ch_delta(ch_is_dead) = 0;
% reduce channel deflection about dead zone (avoid signal jumps)
ch_dead(ch_positive) = jystck.ch_dead_max(ch_positive);
ch_dead(~ch_positive) = jystck.ch_dead_min(~ch_positive);
ch_delta(~ch_is_dead) = ch_delta(~ch_is_dead) - ch_dead(~ch_is_dead);

% compute channel range about dead zone (assure output normalized to -1 ~ 1)
ch_range(ch_positive) = jystck.ch_max(ch_positive) - jystck.ch_trim(ch_positive) - jystck.ch_dead_max(ch_positive);
ch_range(~ch_positive) = jystck.ch_trim(~ch_positive) - jystck.ch_min(~ch_positive) + jystck.ch_dead_min(~ch_positive);

% compute all normalized channel outputs with all corrections
ch_norm = ch_delta ./ ch_range .* ch_sign;
ch_norm(ch_range<=0) = 0;

% put outputs in the correct order
rpytm = ch_norm(jystck.ch_idx);

% pitch is negative for positive stick deflection
rpytm(1:4) = rpytm(jystck.mode_idx) .* [1;-1;1;1];

% bound signals to -1 ~ 1 (normally this should be about right anyway)
rpytm(rpytm>1) = 1;
rpytm(rpytm<-1) = -1;

% define throttle to be between 0 and 1 if stick down is zero throttle
if jystck.zero_throttle == -1
    rpytm(4) = rpytm(4)/2 + 0.5;
end

end