function [lean_cmd, delta] = stickRP2LeanCmd(stick_roll_cmd, stick_pitch_cmd, limiting_mode)
% STICKRP2LEANCMD calculates a limited lean command and the lean direction
%  of the combined roll and pitch stick input.
% 
% Inputs:
%   stick_roll_cmd      stick value, [-1, 1]
%   stick_pitch_cmd     stick value, [-1, 1]
%   limiting_mode       defines how lean_cmd is limited
%                         - 2:          quadratic mode
%                         - otherwise:  circular mode
% 
% Outputs:
%   lean_cmd            limited lean command, [0, 1]
%   delta               direction of the lean angle (see dcm2Lean), rad
% 
% See also:
%   DCM2LEAN

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2024 Jonas Withelm
%   Copyright (C) 2024 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************


delta = atan2(stick_roll_cmd, -stick_pitch_cmd);
lean_cmd = sqrt( stick_roll_cmd^2 + stick_pitch_cmd^2 );

switch limiting_mode
    
    % 2: quadratic mode
    case 2
        lean_max = stickGetMax(delta);
        lean_cmd = lean_cmd / lean_max;
    
    % 1 (and fallback): circular mode
    otherwise
        lean_cmd = min(lean_cmd, 1);
    
end

end





%% LOCAL FUNCTIONS
function lean_max = stickGetMax(delta)
% STICKGETMAX returns the maximum possible value of
%  sqrt(stick_roll_cmd^2 + stick_pitch_cmd^2),
%  when the stick is moved in the direction of delta (lean direction) as
%  far as it can go.
% 
% Inputs:
%   delta       direction of the lean angle (see dcm2Lean), rad
%               delta must be in the closed interval [-pi, pi]
% 
% Outputs:
%   lean_max    maximum possible value of sqrt(roll^2 + pitch^2) for the
%               given lean direction delta

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2024 Jonas Withelm
%   Copyright (C) 2024 TU Braunschweig, Institute of Flight Guidance


delta = abs(delta);

% This must be used if delta could lie outside [-pi, pi]. Here it is not
% needed, because atan2() returns values on the closed interval [-pi, pi].
%delta = mod(delta, pi);

if delta <= 1/4 * pi || delta >= 3/4 * pi
    lean_max = sqrt( 1 + tan(delta)^2 );
    %max = abs( 1/cos(delta) );    % alternative calculation
else
    lean_max = sqrt( 1 + (1/tan(delta))^2 );
    %max = 1/sin(delta);    % alternative calculation
end

end
