function fuselage = fuselageSetLocalInflow( fuselage, xyz_cg  )
% fuselageSetLocalInflow sets the local_inflow struct inside a fuselage
% struct.
% 
% Inputs:
%   fuselage        fuselage struct (see fuselageInit)
%   xyz_cg          vehicle center of gravity position in fuselage coordinate
%                   system (3x1 array), in m
% 
% Outputs:
%   fuselage        fuselage struct (see fuselageInit)
% 
% See also:
%   fuselageInit, fuselageLocalInflowInit

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% local airspeed from rotation
r_cntrl_cg = fuselage.state.geometry.border_pos ...
    - repmat( xyz_cg, 1, size(fuselage.state.geometry.border_pos,2) );
V_Ab = dcmBaFromAeroAngles( fuselage.state.body.alpha, fuselage.state.body.beta ) ...
    * [ fuselage.state.body.V; 0; 0 ];

% total local airspeed including local wind (external) and structure motion
V_border = velocityFromRot( V_Ab, fuselage.state.body.omega, r_cntrl_cg ) ...
    + fuselage.state.geometry_deriv.border_pos_dt - fuselage.state.external.V_Wb;
fuselage.state.aero.local_inflow.V(:) = V_border(:,1:end-1) + diff(V_border,1,2)/2;

% local aerodynamic angles
[ fuselage.state.aero.local_inflow.alpha(:), fuselage.state.aero.local_inflow.beta(:) ] = ...
    aeroAngles( V_border );
fuselage.state.aero.local_inflow.alpha(:) = fuselage.state.aero.local_inflow.alpha ...
    + [ fuselage.state.geometry.alpha(1), fuselage.state.geometry.alpha(1:end-1) ...
    + diff(fuselage.state.geometry.alpha,1,2), fuselage.state.geometry.alpha(end) ];
fuselage.state.aero.local_inflow.beta(:) = fuselage.state.aero.local_inflow.beta ...
    + [ fuselage.state.geometry.beta(1), fuselage.state.geometry.beta(1:end-1) ...
    + diff(fuselage.state.geometry.beta,1,2), fuselage.state.geometry.beta(end) ];

end
