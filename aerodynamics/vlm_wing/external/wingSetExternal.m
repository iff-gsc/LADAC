function external = wingSetExternal( external, V_Wb, V_Wb_dt, atmosphere )


% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

external.V_Wb(:) = V_Wb;
external.V_Wb_dt(:) = V_Wb_dt;
external.atmosphere = atmosphere;

end