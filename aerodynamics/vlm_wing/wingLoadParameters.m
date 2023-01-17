function prm = wingLoadParameters( parameter_file )
% wingLoadParameters creates a parameters struct of a wing.
% 
% Inputs:
%   parameter_file      name of the parameter file (see 
%                       wing_params_default)
% 
% Outputs:
%   prm                 parameters struct as defined by this function
% 
% Example:
%   prm = wingLoadParameters('wing_params_default');
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

run( parameter_file );

prm = wingSetParams( prm );

end