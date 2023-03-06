function external = wingCreateExternal( n_panel, n_panel_x )

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% external additional wind concentrated vectors in body frame, m/s
external.V_Wb = zeros( 3, n_panel, n_panel_x );
% external additional time-derivative of wind in body frame, m/s
external.V_Wb_dt = zeros( 3, n_panel, n_panel_x );
% atmosphere struct
external.atmosphere = isAtmosphere(0);

end