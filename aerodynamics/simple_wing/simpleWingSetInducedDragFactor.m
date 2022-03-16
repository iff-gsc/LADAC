function wing = simpleWingSetInducedDragFactor( wing )

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

b = wing.geometry.b;
c = wing.geometry.c;
e = wing.geometry.e;

Lambda = b/c;
k = 1/(pi*e*Lambda);

wing.polar.params.k = k;

end
