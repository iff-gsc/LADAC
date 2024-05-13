function [] = ardupilotJsonBlockInit( opt_signals_str )
% ardupilotJsonBlockInit initialize Send to ArduPilot SITL (JSON) block
%   This function is called from the initialization of the Simulink block
%   Send to ArduPilot SITL (JSON).
%   It dynamically toggles an additional input to specify optional signals
%   like airspeed or rangefinder distances.
% 
% Syntax:
%   ardupilotJsonBlockInit( opt_signals_str )
% 
% Inputs:
%   opt_signals_str         Optional JSON signals string (1x?char)
%                           With this parameter optional input signals to
%                           the Simulink block can be specified. For
%                           example, to specify airspeed and the rirst
%                           rangefinder distance, use:
%                           '"airspeed":%f,"rng_1":%f'
%                           More info can be found here: [1]
% 
% Outputs:
%   -
% 
% Literature:
%   [1] https://github.com/ArduPilot/ardupilot/tree/master/libraries/SITL/examples/JSON
% 
% See also:
%   ardupilotJson, blockInitToggleInport

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2024 Yannic Beyer
% *************************************************************************

if ~ischar(opt_signals_str)
    error('Mask parameter "Optional signals string" must be a char.');
end

if isempty(opt_signals_str)
    blockInitToggleInport('opt_signals',false);
else
    blockInitToggleInport('opt_signals',true);
end

end