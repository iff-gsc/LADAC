function sens_filt = indiSensFiltLoadParams( filename )
% indiSensFiltLoadParams loads a sensor filter struct for INDI
% 
% Example:
%  sens_filt = indiSensFiltLoadParams( ...
%   'indi_sens_filt__params_default' );
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

run(filename);

end