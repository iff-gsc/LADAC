function wing = simpleWingSetLiftCurveSlope( wing )

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

b = wing.geometry.b;
c = wing.geometry.c;
e = wing.geometry.e;

Lambda = b/c;

% Anderson S. 380
wing.polar.params.C_Lalpha = 2*pi/(1+ 2*pi/(pi*e*Lambda));

end
