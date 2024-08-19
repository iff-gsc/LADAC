function Ma = machNumber( V, a ) %#codegen
% machNumber computes the Mach number
% 
% Inputs:
%   V           air velocity, in m/s
%   a           speed of sound in air, in m/s
% 
% Outputs:
%   Ma          Mach number, in 1
% 

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

Ma = V ./ a;

end