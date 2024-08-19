function downwash = downwashUnstSchedule( downwash, V )
% downwashUnstSchedule update the coefficients of downwash tf depending on
% current airspeed
% 
% Inputs:
%   downwash        downwash struct (see aircraftDownwash)
%   V               airspeed (scalar)
% 
% Outputs:
%   downwash        downwash struct (see aircraftDownwash)
% 
% See also:
%   aircraftDownwash
% 

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

V_rel = V/downwash.V;
len = length(downwash.b);
factor = ( V_rel * ones(1,len) ).^( 1:len );

downwash.b = downwash.b .* factor;
downwash.a = downwash.a .* factor;
downwash.V = V;

end