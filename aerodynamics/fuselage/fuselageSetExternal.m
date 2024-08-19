function fuselage = fuselageSetExternal( fuselage, V_Wb, V_Wb_dt, atmosphere )
% fuselageSetExternal set external struct in fuselage.state struct
%   This function copies the external condition variables into the fuselage
%   struct.
% 
% Inputs:
%   fuselage        fuselage struct (see fuselageInit)
%   V_Wb            wind velocity (see fuselageExternalInit)
%   V_Wb_dt         wind acceleration (see fuselageExternalInit)
%   atmosphere      atmosphere struct (see fuselageExternalInit) 
% 
% Outputs:
%   fuselage        fuselage struct (see fuselageInit)
% 
% See also:
%   fuselageInit, fuselageExternalInit, fuselageCreate, fuselageSetState
% 

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

fuselage.state.external.V_Wb(:) = V_Wb;
fuselage.state.external.V_Wb_dt(:) = V_Wb_dt;
fuselage.state.external.atmosphere = atmosphere;

end