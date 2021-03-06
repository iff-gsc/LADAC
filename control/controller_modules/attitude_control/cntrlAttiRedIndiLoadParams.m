function atc = cntrlAttiRedIndiLoadParams( filename )
% cntrlAttiRedIndiLoadParams loads a reduced attitude controller for INDI
%   struct
% 
% Example:
%  atc = cntrlAttiRedIndiLoadParams( ...
%   'cntrlAttiRedIndi_params_default' );
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

run(filename);

end