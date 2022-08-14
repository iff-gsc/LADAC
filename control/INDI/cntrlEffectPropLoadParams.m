function cntrl_effect = cntrlEffectPropLoadParams( filename )
% cntrlEffectPropLoadParams loads motor-propeller control effectiveness
%   struct
% 
% Example:
%  cntrl_effect = cntrlEffectPropLoadParams( ...
%   'cntrlEffectProp_params_default' );
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

run(filename);

end