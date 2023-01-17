function fuselage = simpleFuselageCreate( filename, V )
% simpleFuselageCreate creates a simple fuselage struct from a parameters
% file.
% 
% Inputs:
%   filename        parameters filename (string), e.g.
%                   'simpleFuselage_params_default'
%   V               volume of the fuselage (scalar), in m^3
% 
% Outputs:
%   fuselage        simple fuselage struct (see simpleFuselageInit)
% 
% See also: simpleFuselageInit, simpleFuselageCl, simpleFuselageCm
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% init
fuselage = simpleFuselageInit();

% load params
fuselage = simpleFuselageLoadParams(fuselage,filename,V);

end