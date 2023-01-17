function airfoil_map = wingAirfoilMapCreateEmpty( )
% wingAirfoilMapCreateEmpty creates an empty airfoil_map struct.
% 
% Outputs:
%   airfoil_map         Struct that contains multiple profile aerodynamics
%                       maps (structs)
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

airfoil_map.map_cl.data=0;
airfoil_map.map_cl.dim=[1,1,1,1,1];
airfoil_map.map_cd.data=0;
airfoil_map.map_cd.dim=[1,1,1,1,1];
airfoil_map.map_cm.data=0;
airfoil_map.map_cm.dim=[1,1,1,1,1];
airfoil_map.alpha.data=0;
airfoil_map.alpha.dim=[1,1];
airfoil_map.Mach.data=0;
airfoil_map.Mach.dim=[1,1];
airfoil_map.Reynolds.data=0;
airfoil_map.Reynolds.dim=[1,1];
airfoil_map.actuator_1.data=0;
airfoil_map.actuator_1.dim=[1,1];
airfoil_map.actuator_2.data=0;
airfoil_map.actuator_2.dim=[1,1];

end

